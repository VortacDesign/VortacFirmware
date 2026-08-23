# vortac_manager.py — coordinator for the Vortac toolchanger.
#
# At klippy:connect this module discovers all [vortac_tool <name>] sections,
# the [vortac_grabber] hardware controller, and the optional
# [vortac_qgl_state] toggle, then registers a T<tool_index> command for
# every tool marked available (ghost sections — autosave leftovers of
# commented-out tools — load as unavailable and are skipped).
#
# Tool change sequence:
#   1. tool_deactivate_gcode (current tool, if any)
#   2. VORTAC_GANTRY_FLAT                  -- frame-flat geometry for docks
#   3. _park_at_dock(current)              -- if a tool is held
#   4. _fetch_from_dock(target)            -- if target is not None
#   5. VORTAC_GANTRY_TILT                  -- bed-flat geometry for printing
#   6. SET_GCODE_OFFSET to target's offsets
#   7. tool_activate_gcode (target tool, if any)
#
# VORTAC_DETECT uses ARGB strobe-by-subtraction to populate the dock
# occupancy map at runtime instead of relying only on the home_dock bootstrap.
#
# Dock ARGB channel semantics: red = power feed (never touched), green =
# pulls the dock_sense pad active low, blue = status LED mirroring green.
# dock_sense readings are ONLY valid while a dock's green channel is on —
# outside of the detection strobe, ALL docks keep G+B on, and every
# sense-dependent operation re-asserts that first (_ensure_sense_active).

import logging
import re


# Hook-on-screws dock geometry. Saved dock positions are the hooked/engage
# position (grabber inside the tool at dock height). The clearance position is
# Z + DOCK_Z_CLEARANCE, used to lift a grabbed tool off the dock screws or
# approach with a held tool before dropping it into the dock.
DOCK_Y_SAFE      = 50.0    # mm, Y clearance for approach/depart
DOCK_Z_CLEARANCE = 4.5     # mm, lift from saved hooked Z to clearance Z
DOCK_APPROACH_F  = 2000    # mm/min, fast move to dock front
DOCK_SLIDE_F     = 500     # mm/min, slow slide-in/out and Z hop

# Unhook verification (fetch only). dock_sense is a pogo pin on a landing
# pad, actively pulled low by the dock's green LED channel — the whole check
# is meaningless unless the dock's sense channels are on (see
# _ensure_sense_active). The dock board rides along on its spring travel
# during the Z lift, so:
#   dock_sense LOW  = pogo contact to the dock pad (tool board is powered
#                     through these pogos while parked!)
#   grab_sense LOW  = grabber physically holding the tool
# Expected good sequence: dock_sense STAYS LOW through the whole lift (board
# follows) and only goes HIGH on the Y backout when the pin slides off the
# pad. Therefore:
#   - dock HIGH *during the lift*  -> tool is tilting/binding, contact lost
#     -> stop within one 0.5mm step, before the pogos tear off.
#   - dock LOW *after a backward step* -> hooks did not release -> stop,
#     re-seat, retry.
#   - grab HIGH at any checkpoint -> grabber lost the tool -> freeze, error.
DOCK_UNHOOK_LIFT_F      = 20    # mm/min, slow Z lift off the dock screws
DOCK_UNHOOK_BACK_F      = 20    # mm/min, checked backward steps
DOCK_UNHOOK_ZSTEP       = 0.5   # mm per checked lift step
DOCK_UNHOOK_STEP        = 1.0   # mm per checked backward step
DOCK_UNHOOK_CHECK_STEPS = 1     # backward steps for dock decoupling check
DOCK_UNHOOK_RETRIES     = 3     # re-seat + lift attempts before reversing


class VortacManager:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.dock_count = config.getint('dock_count', minval=1)
        self.argb_led = config.get('argb_led', default=None)
        # Dock ARGB channel semantics (per dock LED index):
        #   red   = Sicherung/power feed for the tool board — NEVER touched
        #   green = pulls the dock_sense pad active low (sense channel);
        #           sense logic only works while this channel is ON
        #   blue  = status LED, always switched in parallel with green so
        #           the active sense logic is visible at the dock
        self.argb_channel = config.get('argb_channel', default='green').lower()
        self.argb_status_channel = config.get(
            'argb_status_channel', default='blue').lower()
        self.dock_strobe_time = config.getfloat(
            'dock_strobe_time', default=0.10, above=0.0)

        # gcode_macro is needed by [vortac_tool Tn]'s activate/deactivate
        # templates; load it eagerly so the order doesn't matter.
        self.printer.load_object(config, 'gcode_macro')

        # State (populated at klippy:connect)
        self.tools = {}             # tool_id -> VortacTool
        self.current_tool = None    # VortacTool or None
        self.dock_occupancy = {}    # dock_name -> tool_id or None
        self.park_dock = {}         # tool_id -> dock to return the held
                                    # tool to (set on fetch / by detection)
        self.dock_detection_valid = False
        self.calibration_dock = None
        self.calibration_tool = None
        self.grabber = None
        self.qgl_state = None

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'VORTAC_STATUS', self.cmd_VORTAC_STATUS,
            desc="Report current tool, dock map, QGL state")
        gcode.register_command(
            'VORTAC_LOAD', self.cmd_VORTAC_LOAD,
            desc="Manually load tool TOOL=<name>|Tn (no current tool)")
        gcode.register_command(
            'VORTAC_UNLOAD', self.cmd_VORTAC_UNLOAD,
            desc="Manually park the held tool back at its dock")
        gcode.register_command(
            'VORTAC_SET_CURRENT_TOOL', self.cmd_VORTAC_SET_CURRENT_TOOL,
            desc="Set/clear manager's current tool without moving")
        gcode.register_command(
            'VORTAC_DETECT', self.cmd_VORTAC_DETECT,
            desc="Detect grabbed tool and dock occupancy")
        gcode.register_command(
            'VORTAC_SENSE_STATUS', self.cmd_VORTAC_SENSE_STATUS,
            desc="Report raw cached tool sense pin states")
        gcode.register_command(
            'VORTAC_SENSE_MONITOR', self.cmd_VORTAC_SENSE_MONITOR,
            desc="Poll cached sense states for DURATION seconds and "
                 "report every transition (verifies mid-command updates)")
        gcode.register_command(
            'VORTAC_DOCK_STROBE', self.cmd_VORTAC_DOCK_STROBE,
            desc="Manually set one dock strobe channel for debugging")
        gcode.register_command(
            'VORTAC_SELECT_DOCK', self.cmd_VORTAC_SELECT_DOCK,
            desc="Detect/select a dock for calibration")
        gcode.register_command(
            'VORTAC_DOCK_CAL_STATUS', self.cmd_VORTAC_DOCK_CAL_STATUS,
            desc="Report selected calibration dock/tool")
        gcode.register_command(
            'VORTAC_DOCK_CAL_SAVE', self.cmd_VORTAC_DOCK_CAL_SAVE,
            desc="Save current hooked/engage XYZ for selected dock/tool")
        gcode.register_command(
            'VORTAC_DOCK_SAVE_POS', self.cmd_VORTAC_DOCK_SAVE_POS,
            desc="Save current hooked/engage XYZ as TOOL's DOCK position")

        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)
        self.printer.register_event_handler(
            'klippy:ready', self._handle_ready)

    # --------------------------------------------------------------------
    # Connect-time discovery
    # --------------------------------------------------------------------

    def _handle_connect(self):
        gcode = self.printer.lookup_object('gcode')

        # indexed = every non-ghost tool section, including ones explicitly
        # marked available=False — those still own their extruder<N> slot,
        # so index contiguity is validated over this set. Unavailable tools
        # whose index is merely name-derived (ghost autosave sections from
        # the old [vortac_tool T1] naming scheme) are excluded.
        indexed_tools = []
        for name, obj in self.printer.objects.items():
            if not name.startswith('vortac_tool '):
                continue
            if (getattr(obj, 'tool_index', None) is not None
                    and (getattr(obj, 'available', False)
                         or getattr(obj, 'tool_index_explicit', False))):
                indexed_tools.append(obj)
            if not getattr(obj, 'available', False):
                logging.info(
                    "vortac_manager: skipping %s (not available)", name)
                continue
            self.tools[obj.tool_id] = obj
            if obj.home_dock:
                self.dock_occupancy.setdefault(obj.home_dock, obj.tool_id)

        self._validate_tools(indexed_tools)

        self.grabber = self.printer.lookup_object('vortac_grabber', None)
        if self.grabber is None:
            raise self.printer.config_error(
                "vortac_manager: [vortac_grabber] is required but not loaded")

        self.qgl_state = self.printer.lookup_object('vortac_qgl_state', None)

        self._install_probe_guard()

        for tool_id, tool in self.tools.items():
            # The gcode command is T<tool_index> (what slicers emit), NOT the
            # section name — tool ids are display names like "miniGrey".
            # Default-arg trick to capture tool_id per-iteration in the closure.
            gcode.register_command(
                f"T{tool.tool_index}",
                (lambda gcmd, tid=tool_id: self._cmd_change(gcmd, tid)),
                desc=f"Switch to {tool_id} (T{tool.tool_index})")

        logging.info(
            "vortac_manager: registered tools=%s, grabber=%s, qgl_state=%s",
            sorted(self.tools.keys()), bool(self.grabber), bool(self.qgl_state))

    def _validate_tools(self, indexed_tools):
        """Cross-tool sanity checks. Klipper merges duplicate config
        sections silently, so a copied-but-not-fully-renamed tool file
        never errors on its own — it just produces one franken-tool or two
        tools sharing an identity. Catch that here with a clear message."""
        seen = {}
        conflicts = []
        # Klipper hardwires multi-extruder naming to extruder, extruder1,
        # ...: the indices of all non-ghost tools must be exactly 0..N-1.
        # With order-based auto-numbering (include order in tools.cfg) this
        # holds by construction; a gap means explicit tool_index values were
        # mixed in, or a tool section was skipped via skip_sections while
        # another include still counted past it.
        indices = sorted(t.tool_index for t in indexed_tools)
        if indices != list(range(len(indices))):
            conflicts.append(
                f"tool_index values {indices} are not contiguous 0..N-1 "
                f"(check include order in tools.cfg / explicit tool_index "
                f"options)")
        for tid, tool in sorted(self.tools.items()):
            if tool.tool_index is None:
                conflicts.append(
                    f"{tid}: available but has no tool_index (ghost "
                    f"section marked available?)")
            # home_dock is optional: without it, the first VORTAC_DETECT
            # establishes each tool's dock (grabbed tool -> first free
            # calibrated dock); tool changes before that error out with a
            # "run VORTAC_DETECT" message.
            for field, value in (
                    ('tool_index', tool.tool_index),
                    ('mcu_name', tool.mcu_name),
                    ('canbus_uuid', tool.canbus_uuid or None),
                    ('dock_sense_pin', tool.dock_sense_pin),
                    ('grab_sense_pin', tool.grab_sense_pin)):
                if value is None or value == '':
                    continue
                key = (field, value)
                if key in seen:
                    conflicts.append(
                        f"{field}={value!r} shared by {seen[key]} and {tid}")
                else:
                    seen[key] = tid
            if self.printer.lookup_object(f'mcu {tool.mcu_name}', None) is None:
                conflicts.append(
                    f"{tid}: mcu_name '{tool.mcu_name}' has no matching "
                    f"[mcu {tool.mcu_name}] section")
        if conflicts:
            raise self.printer.config_error(
                "vortac_manager: tool identity conflicts (usually a copied "
                "tool file that was not fully renamed): "
                + "; ".join(conflicts))

    # --------------------------------------------------------------------
    # Probe guard
    # --------------------------------------------------------------------

    def _install_probe_guard(self):
        """Refuse every probing operation while a tool is held. The probe
        touch point sits ABOVE the nozzle tip whenever a tool is grabbed,
        so any probing move (QUAD_GANTRY_LEVEL, BED_MESH_CALIBRATE, PROBE,
        PROBE_ACCURACY, ...) would drive the nozzle into the bed before the
        probe can trigger. All of those paths enter the probe object via
        start_probe_session (current Klipper) or run_probe (older Klipper),
        so wrapping both catches everything probe-based."""
        probe = self.printer.lookup_object('probe', None)
        if probe is None:
            return
        for attr in ('start_probe_session', 'run_probe'):
            original = getattr(probe, attr, None)
            if original is None:
                continue

            def guarded(gcmd, _original=original):
                self._ensure_no_tool_for_probing()
                return _original(gcmd)

            setattr(probe, attr, guarded)
        logging.info("vortac_manager: probe guard installed "
                     "(probing refused while a tool is held)")

    def _ensure_no_tool_for_probing(self):
        held = self.current_tool.tool_id if self.current_tool else None
        grabbed = [tid for tid, tool in sorted(self.tools.items())
                   if tool.is_grabbed()]
        if held is None and not grabbed:
            return
        what = held or ', '.join(grabbed)
        raise self.printer.command_error(
            f"Vortac: probing refused — tool {what} is "
            f"{'held' if held else 'reported grabbed by grab_sense'}. "
            f"The probe touch point sits above the nozzle while a tool is "
            f"grabbed; the nozzle would hit the bed first. Park the tool "
            f"(VORTAC_UNLOAD) before QGL/bed mesh/probing.")

    def _handle_ready(self):
        # Defined baseline after every (re)start: sense logic active on all
        # docks, regardless of the neopixel initial_* config values.
        try:
            self._ensure_sense_active()
        except self.printer.command_error as e:
            logging.warning(
                "vortac_manager: could not enable dock sense channels "
                "at startup: %s", e)

    # --------------------------------------------------------------------
    # Dock / tool detection
    # --------------------------------------------------------------------

    def _dock_name(self, dock_index):
        return f"dock{dock_index}"

    def _parse_dock_index(self, dock_name, gcmd):
        dock_name = dock_name.strip().lower()
        if not dock_name.startswith('dock'):
            raise gcmd.error(f"Invalid dock '{dock_name}'")
        try:
            dock_index = int(dock_name[4:])
        except ValueError:
            raise gcmd.error(f"Invalid dock '{dock_name}'")
        if dock_index < 0 or dock_index >= self.dock_count:
            raise gcmd.error(
                f"Dock {dock_name} out of range (dock0..dock{self.dock_count - 1})")
        return dock_index

    def _channel_index(self, name):
        channels = {'red': 0, 'green': 1, 'blue': 2, 'white': 3}
        if name not in channels:
            raise self.printer.command_error(
                f"Unsupported ARGB channel '{name}'")
        return channels[name]

    def _sense_channel_indices(self):
        """Channel indices switched together for the sense logic: the
        sense-pull channel (green) plus the status LED channel (blue)."""
        idx = [self._channel_index(self.argb_channel)]
        status = self._channel_index(self.argb_status_channel)
        if status not in idx:
            idx.append(status)
        return idx

    def _get_led_color_data(self):
        if not self.argb_led:
            raise self.printer.command_error(
                "vortac_manager: argb_led is required for VORTAC_DETECT")
        led = self.printer.lookup_object(f'neopixel {self.argb_led}', None)
        if led is None:
            led = self.printer.lookup_object(self.argb_led, None)
        if led is None:
            raise self.printer.command_error(
                f"vortac_manager: LED object '{self.argb_led}' not found")
        color_data = list(led.get_status(None)['color_data'])
        if len(color_data) < self.dock_count:
            raise self.printer.command_error(
                f"vortac_manager: LED '{self.argb_led}' has {len(color_data)} "
                f"entries, need {self.dock_count}")
        return color_data

    def _set_dock_led_color(self, dock_index, color):
        red, green, blue, white = color
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(
            f"SET_LED LED={self.argb_led} INDEX={dock_index + 1} "
            f"RED={red:.6f} GREEN={green:.6f} BLUE={blue:.6f} "
            f"WHITE={white:.6f} SYNC=0 TRANSMIT=1")

    def _with_strobe_channel(self, color, value):
        """Return `color` with the sense channel AND the status channel set
        to `value` — green (sense pull) and blue (status LED) are always
        switched in parallel; red (power) is never touched."""
        color = list(color)
        for idx in self._sense_channel_indices():
            color[idx] = float(value)
        return tuple(color)

    def _sense_active_on_dock(self, color):
        return all(color[idx] > 0.0 for idx in self._sense_channel_indices())

    def _ensure_sense_active(self, gcmd=None):
        """Make sure every dock has the sense (green) and status (blue)
        channels ON. The dock_sense logic only works while green pulls the
        pad — manual LED changes could have switched it off in between, so
        this is (re)asserted before every sense-dependent operation.
        Returns True if anything had to be corrected."""
        if not self.argb_led:
            return False
        colors = self._get_led_color_data()
        fixed = []
        for i in range(self.dock_count):
            if not self._sense_active_on_dock(colors[i]):
                self._set_dock_led_color(
                    i, self._with_strobe_channel(colors[i], 1.0))
                fixed.append(self._dock_name(i))
        if not fixed:
            return False
        self._dwell_for_sense()
        msg = (f"Vortac: sense channels (G+B) were off on "
               f"{', '.join(fixed)} — re-enabled before continuing")
        if gcmd is not None:
            gcmd.respond_info(msg)
        else:
            logging.info(msg)
        return True

    def _sense_label(self, state):
        if state is None:
            return 'UNKNOWN'
        return 'HIGH' if state else 'LOW'

    def _format_tool_sense_states(self):
        return ', '.join(
            f"{tid}:dock={self._sense_label(tool.dock_sense_state)}"
            f"/grab={self._sense_label(tool.grab_sense_state)}"
            for tid, tool in sorted(self.tools.items()))

    def _format_detection_debug_states(self):
        dock = ', '.join(
            f"{tid}={self._sense_label(tool.dock_sense_state)}"
            for tid, tool in sorted(self.tools.items()))
        grab = ', '.join(
            f"{tid}={self._sense_label(tool.grab_sense_state)}"
            for tid, tool in sorted(self.tools.items()))
        return f"dock[{dock}] grab[{grab}]"

    def _dwell_for_sense(self):
        # Use reactor time so LED updates and button callbacks can be processed
        # inside this gcode command before we read cached sense states.
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + self.dock_strobe_time)

    def _detect_tools(self, gcmd=None):
        debug = bool(gcmd and gcmd.get_int('DEBUG', 0, minval=0, maxval=1))
        debug_lines = []
        original_colors = self._get_led_color_data()
        # Detection baseline: sense (G) + status (B) channels off on all
        # docks. Then each dock is strobed on in turn; the docked tool
        # flips its dock_sense state. Red (power) stays untouched.
        baseline_colors = [
            self._with_strobe_channel(original_colors[i], 0.0)
            for i in range(self.dock_count)
        ]
        detected = {self._dock_name(i): None for i in range(self.dock_count)}
        ambiguous = {}
        try:
            for i, color in enumerate(baseline_colors):
                self._set_dock_led_color(i, color)
            self._dwell_for_sense()
            if debug:
                debug_lines.append(
                    f"baseline all-off: {self._format_detection_debug_states()}")

            baseline_dock = {
                tid: tool.dock_sense_state
                for tid, tool in self.tools.items()
            }
            stuck_low = [
                tid for tid, state in baseline_dock.items()
                if state is False
            ]
            if stuck_low:
                raise self.printer.command_error(
                    "VORTAC_DETECT baseline failed: dock_sense is LOW with "
                    "all dock Tool_id channels off for "
                    f"{', '.join(sorted(stuck_low))}. Check pin polarity, "
                    "pullups, dock wiring, or strobe channel mapping.")
            baseline_grab = {
                tid: tool.grab_sense_state
                for tid, tool in self.tools.items()
            }

            grabbed_ids = [
                tid for tid, state in baseline_grab.items()
                if state is False
            ]

            for dock_index in range(self.dock_count):
                strobe_color = self._with_strobe_channel(
                    baseline_colors[dock_index], 1.0)
                self._set_dock_led_color(dock_index, strobe_color)
                self._dwell_for_sense()
                dock_name = self._dock_name(dock_index)
                if debug:
                    debug_lines.append(
                        f"{dock_name} on: {self._format_detection_debug_states()}")

                candidates = [
                    tid for tid, tool in self.tools.items()
                    if tool.dock_sense_state is False
                ]
                if len(candidates) == 1:
                    detected[dock_name] = candidates[0]
                elif len(candidates) > 1:
                    ambiguous[dock_name] = candidates

                self._set_dock_led_color(dock_index, baseline_colors[dock_index])
                self._dwell_for_sense()

            self.dock_occupancy = detected
            self.dock_detection_valid = True
            if len(grabbed_ids) == 1:
                self.current_tool = self.tools[grabbed_ids[0]]
                self._assign_park_dock(self.current_tool, detected, gcmd)
            elif len(grabbed_ids) == 0:
                self.current_tool = None

            found_ids = set(
                tid for tid in detected.values() if tid is not None)
            found_ids.update(grabbed_ids)
            missing = [
                tid for tid in sorted(self.tools.keys())
                if tid not in found_ids and self.tools[tid].sense_ready()
            ]
            if gcmd is not None:
                self._respond_detection(gcmd, detected, grabbed_ids,
                                        missing, ambiguous, debug_lines)
            return detected
        finally:
            # Normal operating state after detection: sense logic active on
            # ALL docks (G+B on), other channels as they were before.
            for i in range(self.dock_count):
                self._set_dock_led_color(
                    i, self._with_strobe_channel(original_colors[i], 1.0))
            self._dwell_for_sense()

    def _assign_park_dock(self, tool, detected, gcmd):
        """Detection found `tool` in the grabber. Decide where it will be
        parked: keep a previous choice (fetch origin or manual home_dock)
        if that dock is still free, otherwise take the first free dock
        that has calibrated positions for this tool. Warn if none exists —
        parking will fail until a dock is calibrated or freed."""
        tid = tool.tool_id
        existing = self.park_dock.get(tid) or tool.home_dock
        if (existing and existing in detected
                and detected[existing] is None
                and tool.has_dock_pos(existing)):
            self.park_dock[tid] = existing
            return
        for dock in sorted(detected, key=self._parse_dock_sort_key):
            if detected[dock] is None and tool.has_dock_pos(dock):
                self.park_dock[tid] = dock
                if gcmd is not None:
                    gcmd.respond_info(
                        f"Vortac: grabbed tool {tid} will be parked at "
                        f"{dock} (first free calibrated dock)")
                return
        self.park_dock.pop(tid, None)
        msg = (f"Vortac warning: grabbed tool {tid} has no free dock with "
               f"calibrated positions — VORTAC_UNLOAD/tool change will "
               f"fail until a dock is calibrated for it "
               f"(VORTAC_DOCK_SAVE_POS) or a dock is freed")
        if gcmd is not None:
            gcmd.respond_info(msg)
        else:
            logging.warning(msg)

    @staticmethod
    def _parse_dock_sort_key(dock_name):
        try:
            return int(dock_name[4:])
        except (ValueError, IndexError):
            return 1 << 30

    def _respond_detection(self, gcmd, detected, grabbed_ids, missing,
                           ambiguous, debug_lines=None):
        dock_line = ', '.join(
            f"{dock}={tool_id or 'empty'}"
            for dock, tool_id in sorted(detected.items()))
        grabbed = ', '.join(grabbed_ids) if grabbed_ids else 'None'
        missing_line = ', '.join(missing) if missing else 'None'
        msg = (
            f"Vortac detection:\n"
            f"  Docks   : {dock_line}\n"
            f"  Grabbed : {grabbed}\n"
            f"  Missing : {missing_line}")
        if ambiguous:
            amb = ', '.join(
                f"{dock}={','.join(tools)}"
                for dock, tools in sorted(ambiguous.items()))
            msg += f"\n  Ambiguous: {amb}"
        unknown = [
            tid for tid, tool in sorted(self.tools.items())
            if not tool.sense_ready()
        ]
        if unknown:
            msg += f"\n  Unknown sense: {', '.join(unknown)}"
        if debug_lines:
            msg += "\n  Debug:\n    " + "\n    ".join(debug_lines)
        gcmd.respond_info(msg)

    # --------------------------------------------------------------------
    # Tool change state machine
    # --------------------------------------------------------------------

    def _cmd_change(self, gcmd, target_id):
        if target_id not in self.tools:
            raise gcmd.error(
                f"Tool {target_id} not registered or unavailable")
        self._change_to(self.tools[target_id], gcmd)

    def _change_to(self, target, gcmd):
        if self.current_tool is target:
            tid = target.tool_id if target else 'None'
            gcmd.respond_info(f"Already on {tid}")
            return

        gcode = self.printer.lookup_object('gcode')

        # 0. Sense logic sanity: dock_sense only works while the dock's
        # green channel pulls the pad low. Manual LED tinkering may have
        # switched it off — re-assert G+B on all docks before any motion
        # that relies on sense feedback.
        self._ensure_sense_active(gcmd)

        # 1. Deactivate current tool
        if self.current_tool is not None:
            self._render(self.current_tool.tool_deactivate_gcode,
                         tool=self.current_tool, dock=None, gcmd=gcmd)

        # 2. Frame-flat geometry for dock approach
        if self.qgl_state is not None:
            gcode.run_script_from_command('VORTAC_GANTRY_FLAT')

        # 3. Park the held tool (if any). Dock resolution: detection map ->
        # "where I fetched it from" -> optional manual home_dock.
        if self.current_tool is not None:
            tid = self.current_tool.tool_id
            dock = (self._dock_holding(tid) or self.park_dock.get(tid)
                    or self.current_tool.home_dock)
            if dock is None:
                raise self.printer.command_error(
                    f"Vortac: no dock known to park {tid} at — run "
                    f"VORTAC_DETECT (or set home_dock on the tool)")
            self._park_at_dock(self.current_tool, dock, gcmd)
            self.dock_occupancy[dock] = tid
            self.park_dock.pop(tid, None)
            # Parked: if the fetch below fails, we are holding nothing.
            self.current_tool = None

        # 4. Pick up the target tool from its current dock
        if target is not None:
            dock = self._dock_holding(target.tool_id) or target.home_dock
            if dock is None:
                raise self.printer.command_error(
                    f"Vortac: no dock known for {target.tool_id} — run "
                    f"VORTAC_DETECT (or set home_dock on the tool)")
            self._fetch_from_dock(target, dock, gcmd)
            self.dock_occupancy[dock] = None
            # Remember the origin so the tool returns to the same dock.
            self.park_dock[target.tool_id] = dock

        self.current_tool = target

        # 5. Bed-flat geometry for printing
        if self.qgl_state is not None:
            gcode.run_script_from_command('VORTAC_GANTRY_TILT')

        # 6. Switch active extruder, apply target offsets, run activate gcode
        if target is not None:
            if target.extruder_name:
                if self.printer.lookup_object(
                        target.extruder_name, None) is not None:
                    gcode.run_script_from_command(
                        f'ACTIVATE_EXTRUDER EXTRUDER={target.extruder_name}')
                elif gcmd is not None:
                    gcmd.respond_info(
                        f"Vortac warning: extruder '{target.extruder_name}' "
                        f"for {target.tool_id} not found, active extruder "
                        f"unchanged")
            gcode.run_script_from_command(
                f'SET_GCODE_OFFSET X={target.gcode_offset_x} '
                f'Y={target.gcode_offset_y} Z={target.gcode_offset_z} MOVE=0')
            self._render(target.tool_activate_gcode,
                         tool=target, dock=None, gcmd=gcmd)
        else:
            gcode.run_script_from_command(
                'SET_GCODE_OFFSET X=0 Y=0 Z=0 MOVE=0')

    # --------------------------------------------------------------------
    # Dock motion (hardcoded geometry — see constants at top of file)
    # --------------------------------------------------------------------

    def _park_at_dock(self, tool, dock_name, gcmd):
        """Park a held tool: approach lifted, slide in, drop to saved hooked
        Z, disengage the grabber, slide back out."""
        x = tool.get_dock_pos(dock_name, 'x')
        y = tool.get_dock_pos(dock_name, 'y')
        z = tool.get_dock_pos(dock_name, 'z')
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(
            f'G90\n'
            f'G1 X{x} Y{y + DOCK_Y_SAFE} Z{z + DOCK_Z_CLEARANCE} '
            f'F{DOCK_APPROACH_F}\n'
            f'G1 Y{y} F{DOCK_SLIDE_F}\n'
            f'G1 Z{z} F{DOCK_SLIDE_F}')
        self.grabber.disengage(gcmd=gcmd)
        gcode.run_script_from_command(
            f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')

    def _fetch_from_dock(self, tool, dock_name, gcmd):
        """Fetch a docked tool: approach at saved hooked Z, slide in, engage
        the grabber, then unhook slowly with dock_sense verification and
        checked backward steps. On repeated unhook failure, reverse all
        steps (re-seat, disengage, back out empty) and raise."""
        x = tool.get_dock_pos(dock_name, 'x')
        y = tool.get_dock_pos(dock_name, 'y')
        z = tool.get_dock_pos(dock_name, 'z')
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(
            f'G90\n'
            f'G1 X{x} Y{y + DOCK_Y_SAFE} Z{z} F{DOCK_APPROACH_F}\n'
            f'G1 Y{y} F{DOCK_SLIDE_F}')
        self.grabber.engage(gcmd=gcmd)

        if not tool.sense_ready():
            # No sense feedback available — blind unhook, but slow.
            if gcmd is not None:
                gcmd.respond_info(
                    f"Vortac: {tool.tool_id} sense pins not ready, "
                    f"unhooking blind")
            gcode.run_script_from_command(
                f'G1 Z{z + DOCK_Z_CLEARANCE} F{DOCK_UNHOOK_LIFT_F}\n'
                f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
            return

        if not tool.is_grabbed() and gcmd is not None:
            gcmd.respond_info(
                f"Vortac warning: {tool.tool_id} grab_sense does not report "
                f"grabbed after engage")

        for attempt in range(1, DOCK_UNHOOK_RETRIES + 1):
            result = self._try_unhook(tool, y, z, gcmd)
            if result == 'released':
                gcode.run_script_from_command(
                    f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
                self._wait_sense()
                if not tool.is_grabbed():
                    raise self.printer.command_error(
                        f"Vortac: {tool.tool_id} grab_sense went HIGH after "
                        f"leaving {dock_name} — tool may have been lost. "
                        f"Stopped; check the tool before continuing.")
                return
            if result == 'lost_grab':
                # Grabber lost the tool mid-unhook. Do NOT move blindly —
                # the tool's position is unknown. Freeze and demand help.
                raise self.printer.command_error(
                    f"Vortac: grab_sense went HIGH while unhooking "
                    f"{tool.tool_id} from {dock_name} — grabber lost the "
                    f"tool. Motion stopped; intervene manually.")
            if result == 'tilt':
                # Dock pogo contact lost during the lift: tool is binding/
                # tilting on the hooks. Lower straight back down (no Y move
                # has happened yet) to re-seat and restore contact.
                if gcmd is not None:
                    gcmd.respond_info(
                        f"Vortac: {tool.tool_id} dock contact lost during "
                        f"lift at {dock_name} (tilt/bind suspected), "
                        f"lowering to re-seat "
                        f"(attempt {attempt}/{DOCK_UNHOOK_RETRIES})")
                gcode.run_script_from_command(
                    f'G1 Z{z} F{DOCK_UNHOOK_LIFT_F}')
                self._wait_sense()
                self._log_unhook_checkpoint(tool, 'reseat', 0.0)
                if not tool.is_docked():
                    raise self.printer.command_error(
                        f"Vortac: dock contact for {tool.tool_id} at "
                        f"{dock_name} did not return after lowering back "
                        f"to the hooked position. Stopped; check the tool "
                        f"and pogo pins manually.")
                continue
            # result == 'hooked': backward step still shows dock contact,
            # hooks did not release. Re-seat: forward to dock Y, back down.
            if gcmd is not None:
                gcmd.respond_info(
                    f"Vortac: {tool.tool_id} still hooked at {dock_name} "
                    f"(attempt {attempt}/{DOCK_UNHOOK_RETRIES}), re-seating")
            gcode.run_script_from_command(
                f'G1 Y{y} F{DOCK_UNHOOK_BACK_F}\n'
                f'G1 Z{z} F{DOCK_UNHOOK_LIFT_F}')

        # All retries failed — reverse everything: tool stays in its dock,
        # release the grabber and back out empty.
        self.grabber.disengage(gcmd=gcmd)
        gcode.run_script_from_command(
            f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
        raise self.printer.command_error(
            f"Vortac: failed to unhook {tool.tool_id} from {dock_name} "
            f"after {DOCK_UNHOOK_RETRIES} attempts; tool left in dock, "
            f"grabber disengaged")

    def _try_unhook(self, tool, y, z, gcmd):
        """One unhook attempt from the hooked position.

        Stepwise slow lift, during which dock_sense must STAY LOW (the
        spring-loaded dock board rides along; losing pogo contact here
        means the tool is tilting on stuck hooks). Then checked backward
        steps, where dock_sense going HIGH is the clean release.

        Returns 'released', 'hooked' (dock still LOW after the checked
        backward steps), 'tilt' (dock contact lost during the lift; no Y
        motion has happened yet), or 'lost_grab' (grab_sense went HIGH).
        Leaves the toolhead wherever the last checked step ended."""
        gcode = self.printer.lookup_object('gcode')
        cur_z = z
        while cur_z < z + DOCK_Z_CLEARANCE - 1e-6:
            cur_z = min(cur_z + DOCK_UNHOOK_ZSTEP, z + DOCK_Z_CLEARANCE)
            gcode.run_script_from_command(
                f'G1 Z{cur_z:.3f} F{DOCK_UNHOOK_LIFT_F}')
            self._wait_sense()
            self._log_unhook_checkpoint(tool, 'lift', cur_z - z)
            if not tool.is_grabbed():
                return 'lost_grab'
            if not tool.is_docked():
                return 'tilt'
        cur_y = y
        for _ in range(DOCK_UNHOOK_CHECK_STEPS):
            cur_y += DOCK_UNHOOK_STEP
            gcode.run_script_from_command(
                f'G1 Y{cur_y:.3f} F{DOCK_UNHOOK_BACK_F}')
            self._wait_sense()
            self._log_unhook_checkpoint(tool, 'backout', cur_y - y)
            if not tool.is_grabbed():
                return 'lost_grab'
            if not tool.is_docked():
                return 'released'
        return 'hooked'

    def _log_unhook_checkpoint(self, tool, phase, offset):
        """Trace every unhook checkpoint to klippy.log for post-mortem
        analysis of the sense signals during a real load."""
        logging.info(
            "vortac unhook %s: %s +%.3fmm dock=%s grab=%s",
            tool.tool_id, phase, offset,
            self._sense_label(tool.dock_sense_state),
            self._sense_label(tool.grab_sense_state))

    def _wait_sense(self):
        """Wait for motion to finish, then let sense callbacks run so the
        cached dock/grab states reflect the new position."""
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        self._dwell_for_sense()

    def _dock_holding(self, tool_id):
        for dock, tid in self.dock_occupancy.items():
            if tid == tool_id:
                return dock
        return None

    def _resolve_tool_id(self, requested):
        """Resolve a user-supplied tool reference to a registered tool_id.

        Accepts the exact tool name ("miniGrey"), a case-insensitive name,
        "T<index>", or a bare index. Ids coming from internal maps
        (dock_occupancy) hit the exact match. Never mangles the input —
        tool names are arbitrary (they are the include_with namespace)."""
        req = requested.strip()
        if req in self.tools:
            return req
        lowered = {tid.lower(): tid for tid in self.tools}
        if req.lower() in lowered:
            return lowered[req.lower()]
        m = re.fullmatch(r'[Tt]?(\d+)', req)
        if m:
            idx = int(m.group(1))
            for tid, tool in self.tools.items():
                if tool.tool_index == idx:
                    return tid
        return None

    def _get_tool(self, tool_id, gcmd):
        resolved = self._resolve_tool_id(tool_id)
        if resolved is None:
            known = ', '.join(sorted(self.tools)) or '(none)'
            raise gcmd.error(
                f"Tool {tool_id!r} not registered or unavailable "
                f"(known tools: {known})")
        return self.tools[resolved]

    def _ensure_gantry_flat(self, gcmd):
        if self.qgl_state is not None and self.qgl_state.state != 'flat':
            raise gcmd.error(
                f"Refuse to save dock position: gantry is "
                f"{self.qgl_state.state!r}; run VORTAC_GANTRY_FLAT first "
                f"(dock geometry is only valid frame-flat)")

    def _save_dock_pos(self, tool, dock, gcmd):
        """Save current toolhead position as the dock's hooked/engage XYZ."""
        self._ensure_gantry_flat(gcmd)
        self._parse_dock_index(dock, gcmd)
        toolhead = self.printer.lookup_object('toolhead')
        x, y, z = toolhead.get_position()[:3]
        tool.save_dock_pos(dock, x, y, z)
        gcmd.respond_info(
            f"Saved {tool.tool_id} @ {dock}: "
            f"X={x:.4f} Y={y:.4f} Z={z:.4f}.  Run SAVE_CONFIG to persist.")

    def _render(self, template, tool, dock, gcmd):
        if template is None:
            return
        ctx = {
            **template.create_template_context(),
            'tool': tool,
            'dock': dock,
            'params': gcmd.get_command_parameters() if gcmd else {},
        }
        template.run_gcode_from_command(ctx)

    # --------------------------------------------------------------------
    # Manual commands
    # --------------------------------------------------------------------

    def cmd_VORTAC_STATUS(self, gcmd):
        cur = self.current_tool.tool_id if self.current_tool else 'None'
        qgl = self.qgl_state.state if self.qgl_state else 'n/a'
        tools_line = (', '.join(
            f"{tid} (T{tool.tool_index})"
            for tid, tool in sorted(self.tools.items()))
            if self.tools else '(none registered)')
        occ = ', '.join(
            f"{d}={tid or 'empty'}"
            for d, tid in sorted(self.dock_occupancy.items()))
        cal_tool = self.calibration_tool.tool_id if self.calibration_tool else 'None'
        cal_dock = self.calibration_dock or 'None'
        gcmd.respond_info(
            f"Vortac status:\n"
            f"  Current tool : {cur}\n"
            f"  Tools loaded : {tools_line}\n"
            f"  Dock map     : {occ or '(unknown)'}\n"
            f"  Detection    : {'valid' if self.dock_detection_valid else 'bootstrap'}\n"
            f"  Cal selected : {cal_dock} / {cal_tool}\n"
            f"  QGL state    : {qgl}")

    def cmd_VORTAC_LOAD(self, gcmd):
        target = self._get_tool(gcmd.get('TOOL'), gcmd)
        if self.current_tool is not None:
            raise gcmd.error(
                f"Already holding {self.current_tool.tool_id}; "
                f"unload first or use {target.tool_id} to swap directly")
        self._change_to(target, gcmd)

    def cmd_VORTAC_UNLOAD(self, gcmd):
        if self.current_tool is None:
            raise gcmd.error("No tool currently held")
        self._change_to(None, gcmd)

    def cmd_VORTAC_SET_CURRENT_TOOL(self, gcmd):
        """Set logical manager state only; does not move or touch the grabber."""
        tool_arg = gcmd.get('TOOL', None)
        clear = gcmd.get('CLEAR', None)
        if clear is not None:
            self.current_tool = None
            gcmd.respond_info("Cleared current Vortac tool")
            return
        if tool_arg is None:
            raise gcmd.error("Missing TOOL=<name>|Tn or CLEAR=1")
        self.current_tool = self._get_tool(tool_arg, gcmd)
        gcmd.respond_info(
            f"Set current Vortac tool to {self.current_tool.tool_id} "
            f"(no motion performed)")

    def cmd_VORTAC_DETECT(self, gcmd):
        self._detect_tools(gcmd)

    def cmd_VORTAC_SENSE_STATUS(self, gcmd):
        if not self.tools:
            gcmd.respond_info("No Vortac tools registered")
            return
        lines = ["Vortac sense states:"]
        for tid, tool in sorted(self.tools.items()):
            lines.append(
                f"  {tid}: dock={self._sense_label(tool.dock_sense_state)} "
                f"grab={self._sense_label(tool.grab_sense_state)} "
                f"dock_pin={tool.dock_sense_pin or 'n/a'} "
                f"grab_pin={tool.grab_sense_pin or 'n/a'}")
        if self.argb_led:
            colors = self._get_led_color_data()
            states = ', '.join(
                f"{self._dock_name(i)}="
                f"{'on' if self._sense_active_on_dock(colors[i]) else 'OFF'}"
                for i in range(self.dock_count))
            lines.append(f"  Dock sense channels (G+B): {states}")
            if any(not self._sense_active_on_dock(colors[i])
                   for i in range(self.dock_count)):
                lines.append(
                    "  WARNING: dock_sense is only valid while a dock's "
                    "sense channels are on (green pulls the pad low)")
        gcmd.respond_info('\n'.join(lines))

    def cmd_VORTAC_SENSE_MONITOR(self, gcmd):
        """Live-verify that sense callbacks are processed during
        reactor.pause inside a running gcode command — the exact mechanism
        the unhook checkpoints rely on. While this runs, other gcode is
        queued, so change states physically (lift/wiggle the tool)."""
        duration = gcmd.get_float('DURATION', 10.0, above=0.0)
        interval = gcmd.get_float(
            'INTERVAL', self.dock_strobe_time, above=0.0)
        if not self.tools:
            raise gcmd.error("No Vortac tools registered")
        reactor = self.printer.get_reactor()
        start = reactor.monotonic()
        last = {
            tid: (tool.dock_sense_state, tool.grab_sense_state)
            for tid, tool in self.tools.items()
        }
        gcmd.respond_info(
            f"Monitoring sense states for {duration:.1f}s "
            f"(poll every {interval * 1000:.0f}ms). "
            f"Start: {self._format_tool_sense_states()}")
        transitions = 0
        while reactor.monotonic() - start < duration:
            reactor.pause(reactor.monotonic() + interval)
            for tid, tool in sorted(self.tools.items()):
                cur = (tool.dock_sense_state, tool.grab_sense_state)
                if cur == last[tid]:
                    continue
                elapsed = reactor.monotonic() - start
                gcmd.respond_info(
                    f"[{elapsed:6.2f}s] {tid}: "
                    f"dock {self._sense_label(last[tid][0])}"
                    f"->{self._sense_label(cur[0])}, "
                    f"grab {self._sense_label(last[tid][1])}"
                    f"->{self._sense_label(cur[1])}")
                last[tid] = cur
                transitions += 1
        gcmd.respond_info(
            f"Monitor done: {transitions} transition(s) in {duration:.1f}s. "
            f"End: {self._format_tool_sense_states()}")

    def cmd_VORTAC_DOCK_STROBE(self, gcmd):
        dock = gcmd.get('DOCK').strip().lower()
        dock_index = self._parse_dock_index(dock, gcmd)
        value = gcmd.get_float('VALUE', minval=0.0, maxval=1.0)
        color = self._get_led_color_data()[dock_index]
        self._set_dock_led_color(
            dock_index, self._with_strobe_channel(color, value))
        gcmd.respond_info(
            f"Set {dock} sense channels ({self.argb_channel}+"
            f"{self.argb_status_channel}) to {value:.3f}")

    def cmd_VORTAC_SELECT_DOCK(self, gcmd):
        dock = gcmd.get('DOCK').strip().lower()
        self._parse_dock_index(dock, gcmd)
        if gcmd.get_int('DETECT', 1, minval=0, maxval=1):
            self._detect_tools(gcmd)
        elif not self.dock_detection_valid:
            raise gcmd.error("No dock detection map; run VORTAC_DETECT first")
        else:
            # No detection pass (which would restore them) — make sure the
            # sense channels are on before calibration relies on them.
            self._ensure_sense_active(gcmd)

        tool_id = self.dock_occupancy.get(dock)
        if tool_id is None:
            self.calibration_dock = None
            self.calibration_tool = None
            raise gcmd.error(f"{dock} has no detected tool")
        self.calibration_dock = dock
        self.calibration_tool = self._get_tool(tool_id, gcmd)
        gcmd.respond_info(
            f"Selected {dock} with {self.calibration_tool.tool_id} "
            f"for Vortac dock calibration")

    def cmd_VORTAC_DOCK_CAL_STATUS(self, gcmd):
        selected_tool = (self.calibration_tool.tool_id
                         if self.calibration_tool else 'None')
        selected_dock = self.calibration_dock or 'None'
        dock_line = ', '.join(
            f"{dock}={tool_id or 'empty'}"
            for dock, tool_id in sorted(self.dock_occupancy.items()))
        gcmd.respond_info(
            f"Vortac dock calibration:\n"
            f"  Selected dock : {selected_dock}\n"
            f"  Selected tool : {selected_tool}\n"
            f"  Detection map : {dock_line or '(unknown)'}")

    def cmd_VORTAC_DOCK_CAL_SAVE(self, gcmd):
        if self.calibration_dock is None or self.calibration_tool is None:
            raise gcmd.error(
                "No calibration dock selected; run VORTAC_SELECT_DOCK first")
        self._save_dock_pos(
            self.calibration_tool, self.calibration_dock, gcmd)

    def cmd_VORTAC_DOCK_SAVE_POS(self, gcmd):
        dock = gcmd.get('DOCK').strip()
        tool_arg = gcmd.get('TOOL', None)
        tool = self._get_tool(tool_arg, gcmd) if tool_arg is not None \
               else self.current_tool
        if tool is None:
            raise gcmd.error(
                "No tool selected; use TOOL=<name>|Tn or "
                "VORTAC_SET_CURRENT_TOOL")
        self._save_dock_pos(tool, dock, gcmd)

    # --------------------------------------------------------------------
    # Status (for templates / dashboards)
    # --------------------------------------------------------------------

    def get_status(self, eventtime):
        return {
            'current_tool': (self.current_tool.tool_id
                             if self.current_tool else None),
            'tools': sorted(self.tools.keys()),
            'dock_occupancy': dict(self.dock_occupancy),
            'dock_detection_valid': self.dock_detection_valid,
            'calibration_dock': self.calibration_dock,
            'calibration_tool': (self.calibration_tool.tool_id
                                 if self.calibration_tool else None),
            'qgl_state': self.qgl_state.state if self.qgl_state else None,
        }


def load_config(config):
    return VortacManager(config)
