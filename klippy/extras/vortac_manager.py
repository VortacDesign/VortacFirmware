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
# Dock ARGB channel roles (all three are driven from _apply_dock_leds):
#   green = pulls the dock_sense pad active low. dock_sense readings are ONLY
#           valid while this channel is on — outside of the detection strobe
#           every dock keeps it on, and every sense-dependent operation
#           re-asserts it first (_ensure_sense_active).
#   red   = gates the tool board's supply through the dock pogos. INVERTED on
#           Vortac hardware: red HIGH cuts the supply, red 0.0 = powered. That
#           polarity is a hard requirement, not a preference — see below.
#   blue  = status LED, driven by argb_status_mode (sense | power | detected).
#
# DARK POWER MANAGEMENT (dock_power_mode)
# The pogo contact between tool board and dock is made and broken by the Y
# slide at the dock — the spring-loaded dock board rides along through the
# whole Z travel, so Z never opens it. Every supply switch therefore brackets
# a Y move, never a Z move:
#   park  : slide in (contact closes dead) -> verify dock_sense -> power ON
#           -> checked Z drop -> disengage -> slide out
#   fetch : slide in -> engage -> checked Z lift -> verify grab_sense
#           -> power OFF -> slide out (contact opens dead)
# In both cases the tool board is fed from the other side while the dock side
# switches: the grabber carries it during fetch, the dock carries it after
# park. There is never a moment where both feeds are down.
#
# HARD CONSTRAINT: dock_sense/grab_sense live on the TOOL mcu, and Klipper has
# no notion of an optional mcu. A tool board that goes dark drops off the CAN
# bus and takes klippy down with it. Hence:
#   - only a dock POSITIVELY known to be empty is ever de-energized;
#     anything uncertain stays live (_policy_power).
#   - the unpowered state must be the LED's HIGH state (dock_power_invert),
#     because klippy:mcu_identify runs BEFORE klippy:connect ever programs the
#     neopixel — at startup the tool boards must already be alive, so "no LED
#     data yet" has to mean "powered".
#   - WS2812 latch their last value across a klippy RESTART, so RESTART and
#     FIRMWARE_RESTART are wrapped to re-energize every dock first
#     (_install_restart_guard). A full power cycle clears the latch and is the
#     unconditional recovery path.

import logging
import re


# Hook-on-screws dock geometry. Saved dock positions are the hooked/engage
# position (grabber inside the tool at dock height). The clearance position is
# Z + DOCK_Z_CLEARANCE, used to lift a grabbed tool off the dock screws or
# approach with a held tool before dropping it into the dock.
DOCK_Y_SAFE      = 50.0    # mm, Y clearance for approach/depart
DOCK_Z_CLEARANCE = 5.0     # mm, lift from saved hooked Z to clearance Z
DOCK_APPROACH_F  = 2000    # mm/min, fast move to dock front
DOCK_SLIDE_F     = 400     # mm/min, Y slide-in/out only (Z drop/lift at the
                           # dock is always the checked DOCK_UNHOOK_* phase)

# Hook/unhook verification (used by BOTH park and fetch: the checked
# stepwise Z drop when hooking in mirrors the checked lift when unhooking,
# with the same step size and feedrate). dock_sense is a pogo pin on a landing
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
#     -> stop within one DOCK_UNHOOK_ZSTEP, before the pogos tear off.
#   - dock LOW *after a backward step* -> hooks did not release -> stop,
#     re-seat, retry.
#   - grab HIGH at any checkpoint -> grabber lost the tool -> freeze, error.
DOCK_UNHOOK_LIFT_F      = 40    # mm/min, slow Z lift off the dock screws
DOCK_UNHOOK_BACK_F      = 40    # mm/min, checked backward steps
DOCK_UNHOOK_ZSTEP       = 1.0   # mm per checked lift step
DOCK_UNHOOK_STEP        = 1.5   # mm per checked backward step
DOCK_UNHOOK_CHECK_STEPS = 1     # backward steps for dock decoupling check
DOCK_UNHOOK_RETRIES     = 3     # re-seat + lift attempts before reversing


class VortacManager:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.dock_count = config.getint('dock_count', minval=1)
        self.argb_led = config.get('argb_led', default=None)
        # See the channel-role block at the top of this file.
        self.argb_channel = config.get('argb_channel', default='green').lower()
        self.argb_status_channel = config.get(
            'argb_status_channel', default='blue').lower()
        # What the status LED shows:
        #   sense    = mirrors the sense pull (strobes along with detection)
        #   power    = lit while the dock feeds the tool board (inverse of red)
        #   detected = full for a detected tool, dim for a confirmed empty
        #              dock, off while the dock map is not trustworthy
        self.argb_status_mode = config.getchoice(
            'argb_status_mode',
            {m: m for m in ('sense', 'power', 'detected')}, 'sense')
        self.argb_status_dim = config.getfloat(
            'argb_status_dim', default=0.15, minval=0.0, maxval=1.0)
        self.dock_strobe_time = config.getfloat(
            'dock_strobe_time', default=0.10, above=0.0)

        # --- dock power (dark power management) -------------------------
        #   off       = never touch the power channel
        #   occupancy = de-energize docks positively known to be empty
        #   handover  = additionally cut the supply before the fetch backout,
        #               so the pogos separate dead (needs the grabber to feed
        #               the held tool — verified at runtime, see
        #               _power_off_before_backout)
        self.dock_power_mode = config.getchoice(
            'dock_power_mode',
            {m: m for m in ('off', 'occupancy', 'handover')}, 'off')
        self.dock_power_channel = config.get(
            'dock_power_channel', default='red').lower()
        # True (default, Vortac hardware): channel HIGH cuts the supply and
        # 0.0 means powered. Do not flip this without reading the startup
        # ordering note at the top of this file.
        self.dock_power_invert = config.getboolean(
            'dock_power_invert', default=True)
        self.dock_power_settle = config.getfloat(
            'dock_power_settle', default=0.25, minval=0.0)
        # A manual SET_LED that raises the supply channel on an occupied
        # dock is only corrected at the next sense-dependent operation — but
        # a service restart (systemctl / SIGTERM) can happen first, and THAT
        # path no klippy code can intercept: the WS2812 latches the tampered
        # state and the board is dark at the next mcu_identify. The watchdog
        # closes the window by re-asserting the supply channel against the
        # policy every few seconds. 0 disables it.
        self.dock_power_watchdog = config.getfloat(
            'dock_power_watchdog', default=5.0, minval=0.0)
        # Channel names are only dereferenced at runtime (_channel_index),
        # and the klippy:ready reconcile downgrades that failure to a log
        # warning — validate here so a typo is a startup error instead of a
        # printer that silently runs without sense/power management.
        _channels = ('red', 'green', 'blue', 'white')
        if self.argb_channel not in _channels:
            raise config.error(
                f"vortac_manager: unknown argb_channel "
                f"'{self.argb_channel}' (expected one of "
                f"{', '.join(_channels)})")
        for _name, _value in (
                ('argb_status_channel', self.argb_status_channel),
                ('dock_power_channel', self.dock_power_channel)):
            if _value not in _channels and _value not in ('', 'none'):
                raise config.error(
                    f"vortac_manager: unknown {_name} '{_value}' "
                    f"(expected one of {', '.join(_channels)}, or none)")
        if self.dock_power_mode != 'off' and (
                not self.argb_led
                or self.dock_power_channel in ('', 'none')):
            raise config.error(
                f"vortac_manager: dock_power_mode "
                f"'{self.dock_power_mode}' needs argb_led and a "
                f"dock_power_channel — without them no supply can actually "
                f"be switched and the mode would silently do nothing")
        if self.dock_power_channel not in ('', 'none'):
            for name, other in (('argb_channel', self.argb_channel),
                                ('argb_status_channel',
                                 self.argb_status_channel)):
                if self.dock_power_channel == other:
                    raise config.error(
                        f"vortac_manager: dock_power_channel "
                        f"'{self.dock_power_channel}' collides with {name} — "
                        f"the supply gate and the sense/status LED cannot "
                        f"share one channel")
        # Run VORTAC_DETECT automatically once klippy is ready, so the dock
        # map exists right after every (re)start without a manual step.
        self.auto_detect = config.getboolean('auto_detect', default=True)
        self.auto_detect_delay = config.getfloat(
            'auto_detect_delay', default=3.0, minval=0.0)

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
        # Docks whose occupancy the last detection could not pin down
        # (ambiguous, or some tool went unaccounted for). Never de-energized,
        # and shown as "off" by the 'detected' status mode.
        self.dock_uncertain = set()
        # Tool sections klippy needs on the CAN bus but this module cannot
        # track (unavailable, yet their [mcu ...] is included). Detection can
        # never locate them, so no dock may ever be de-energized.
        self.unmanaged_tools = []
        # Transient supply overrides during a handover, dock -> bool. Takes
        # precedence over the occupancy-derived policy.
        self.dock_power_override = {}
        # Debounce for the watchdog: at most one fix in flight at a time.
        self._watchdog_fix_pending = False
        # Set once if the dock's sense pull turns out to die with the dock
        # supply — then dock_sense cannot be trusted while the dock is dead
        # and the dead-break half of the handover is disabled.
        self.sense_needs_dock_power = False

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
            'VORTAC_DOCK_POWER', self.cmd_VORTAC_DOCK_POWER,
            desc="Report or override the dock supply "
                 "(DOCK=dockN VALUE=0|1 [ONLY=1] [FORCE=1])")
        gcode.register_command(
            'VORTAC_STATUS_LED', self.cmd_VORTAC_STATUS_LED,
            desc="Switch what the dock status LED shows "
                 "(MODE=sense|power|detected)")
        gcode.register_command(
            '_VORTAC_DOCK_POWER_SYNC', self.cmd_VORTAC_DOCK_POWER_SYNC,
            desc="Re-assert the dock supply channel against the policy "
                 "(used by the dock power watchdog)")
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
                # An unavailable tool whose board is still on the bus is one
                # klippy REQUIRES but this module can never account for:
                # detection cannot see it, so its dock must never be
                # de-energized. Distinguishing that from a harmless ghost (a
                # commented-out tool file, whose [mcu ...] is gone too) needs
                # two signals, because either one alone gets it wrong:
                #   - a resolvable mcu_name proves the board is configured,
                #     but a typo'd/absent mcu_name on a real disabled tool
                #     would slip through;
                #   - a canbus_uuid is only ever injected by include_with
                #     from a live tool file, so it catches exactly that case,
                #     while an old-scheme ghost ([vortac_tool T1]) has none
                #     even though it does derive a default mcu_name.
                mcu_name = getattr(obj, 'mcu_name', None)
                has_mcu = bool(mcu_name) and self.printer.lookup_object(
                    f'mcu {mcu_name}', None) is not None
                if has_mcu or getattr(obj, 'canbus_uuid', ''):
                    self.unmanaged_tools.append(obj.tool_id)
                continue
            self.tools[obj.tool_id] = obj
            if obj.home_dock:
                self.dock_occupancy.setdefault(obj.home_dock, obj.tool_id)

        if self.unmanaged_tools:
            logging.warning(
                "vortac_manager: %s unavailable but their mcu is live — dock "
                "power will stay on everywhere, since detection cannot tell "
                "which dock holds them", ', '.join(self.unmanaged_tools))

        self._validate_tools(indexed_tools)

        self.grabber = self.printer.lookup_object('vortac_grabber', None)
        if self.grabber is None:
            raise self.printer.config_error(
                "vortac_manager: [vortac_grabber] is required but not loaded")

        self.qgl_state = self.printer.lookup_object('vortac_qgl_state', None)

        self._install_probe_guard()
        self._install_restart_guard()

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

    # --------------------------------------------------------------------
    # Restart guard
    # --------------------------------------------------------------------

    def _install_restart_guard(self):
        """Energize every dock before a klippy restart.

        WS2812 hold their last latched value across a klippy or firmware
        restart, so a dock left dead would still be dead during the next
        klippy:mcu_identify — and its tool board would be missing from the
        CAN bus before any Python code gets a chance to switch it back on.
        Only a full power cycle clears the latch by itself, so the graceful
        restart paths are wrapped here.

        Limitation worth knowing: this guard only exists on the gcode path.
        A service restart (systemctl restart klipper / SIGTERM — the standard
        step after every git pull) kills the host process without running any
        klippy code, so nothing can energize the docks on that path. The
        policy keeps every occupied dock on at rest, so the only way to be
        bitten is a supply tampered with shortly before the service restart —
        and the dock power watchdog reverts exactly that within seconds.

        Also: in a SHUTDOWN state this guard is a no-op.
        SET_LED is not registered when_not_ready, so the write is refused and
        _force_all_docks_powered swallows it (deliberately — recovering the
        printer beats switching an LED). It therefore covers the graceful
        restart, but not the case that produced the shutdown. There the only
        recovery is the power cycle, which is safe by construction because
        the unprogrammed LED state means "powered"."""
        if self._power_index() is None:
            return
        gcode = self.printer.lookup_object('gcode')
        for cmd in ('RESTART', 'FIRMWARE_RESTART'):
            original = gcode.register_command(cmd, None)
            if original is None:
                continue

            def guarded(gcmd, _original=original, _cmd=cmd):
                # Must never block the restart itself: in a shutdown state
                # SET_LED is refused, and getting the printer back up beats
                # switching an LED. _force_all_docks_powered swallows that.
                self._force_all_docks_powered(_cmd)
                return _original(gcmd)

            # when_not_ready=True is how Klipper registers these two — drop
            # it and RESTART/FIRMWARE_RESTART would stop working after a
            # shutdown, i.e. exactly when they are needed most.
            gcode.register_command(
                cmd, guarded, when_not_ready=True,
                desc=f"{cmd} (Vortac: energizes all docks first so no tool "
                     f"board is dark at the next mcu identify)")
        logging.info("vortac_manager: restart guard installed "
                     "(all docks energized before RESTART/FIRMWARE_RESTART)")

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

    def _check_led_channels(self):
        """Warn if a configured channel is not physically transmitted.

        Klipper stores all four components regardless of the chain's
        color_order, but only transmits the ones the order names. A
        dock_power_channel the chain never sends would leave the supply
        permanently at its hardware default while this module, VORTAC_STATUS
        and both dashboards all report a supply state that does not exist —
        so it is worth naming out loud. Only a warning: color_order is
        per-LED and the manager has no business refusing to start over a
        cosmetic channel."""
        if not self.argb_led:
            return
        led = (self.printer.lookup_object(f'neopixel {self.argb_led}', None)
               or self.printer.lookup_object(self.argb_led, None))
        order = getattr(led, 'color_order', None)
        if not order:
            return
        for i in range(self.dock_count):
            letters = order[i] if i < len(order) else order[-1]
            for name, channel in (('argb_channel', self.argb_channel),
                                  ('argb_status_channel',
                                   self.argb_status_channel),
                                  ('dock_power_channel',
                                   self.dock_power_channel)):
                if channel in ('', 'none'):
                    continue
                if channel[0].upper() not in letters.upper():
                    logging.warning(
                        "vortac_manager: %s '%s' is not in %s's color_order "
                        "'%s' for %s — that channel is never transmitted, so "
                        "it cannot switch anything", name, channel,
                        self.argb_led, letters, self._dock_name(i))

    def _handle_ready(self):
        # Defined baseline after every (re)start: sense pull active on all
        # docks, regardless of the neopixel initial_* config values. The
        # supply stays untouched here — every tool board is alive by now
        # (klippy:mcu_identify has passed) and nothing may be switched off
        # before the auto-detect below has established which dock is empty.
        try:
            self._ensure_sense_active()
        except self.printer.command_error as e:
            logging.warning(
                "vortac_manager: could not enable dock sense channels "
                "at startup: %s", e)
        self._start_power_watchdog()
        # Purely diagnostic, so it runs AFTER the functional reconcile and
        # can never break startup: an exception here would otherwise abort
        # klippy:ready, leaving the sense pull at its initial_* value (off)
        # and the auto-detect below unscheduled — i.e. a printer that finds
        # no tools, caused by a cosmetic check.
        try:
            self._check_led_channels()
        except Exception:
            logging.exception(
                "vortac_manager: LED channel check failed (diagnostic only)")
        if self.auto_detect:
            # Deferred a few seconds so the tool boards' button callbacks
            # have delivered their first sense readings. Runs through the
            # gcode queue (same pattern as [delayed_gcode]) so output lands
            # in the console and a failure cannot take klippy down.
            reactor = self.printer.get_reactor()
            reactor.register_callback(
                self._auto_detect_cb,
                reactor.monotonic() + self.auto_detect_delay)

    def _auto_detect_cb(self, eventtime):
        gcode = self.printer.lookup_object('gcode')
        try:
            gcode.run_script("VORTAC_DETECT")
        except Exception as e:
            logging.warning("vortac_manager: startup auto-detect failed: %s",
                            e)
            try:
                gcode.respond_info(
                    f"Vortac: startup auto-detect failed ({e}). "
                    "Run VORTAC_DETECT manually.")
            except Exception:
                pass

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

    def _sense_index(self):
        """Index of the sense-pull channel (green) — the only channel that
        is electrically load-bearing for dock_sense."""
        return self._channel_index(self.argb_channel)

    def _status_index(self):
        """Index of the status LED channel, or None when it is disabled or
        shares the sense channel (then the sense value already drives it)."""
        if self.argb_status_channel in ('', 'none'):
            return None
        idx = self._channel_index(self.argb_status_channel)
        return None if idx == self._sense_index() else idx

    def _power_index(self):
        """Index of the supply gate channel, or None when dock power
        management is disabled — then the channel is left untouched."""
        if (self.dock_power_mode == 'off'
                or self.dock_power_channel in ('', 'none')
                or not self.argb_led):
            # Without an LED chain nothing is actually switched, and claiming
            # a managed supply would make VORTAC_STATUS and the dashboards
            # report a state that does not exist.
            return None
        return self._channel_index(self.dock_power_channel)

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

    def _write_dock_colors(self, colors):
        """Write {dock_index: rgbw} as one batch — TRANSMIT is set only on
        the last entry, so a full reconcile pass refreshes the chain once."""
        gcode = self.printer.lookup_object('gcode')
        items = sorted(colors.items())
        for n, (dock_index, color) in enumerate(items):
            red, green, blue, white = color
            transmit = 1 if n == len(items) - 1 else 0
            gcode.run_script_from_command(
                f"SET_LED LED={self.argb_led} INDEX={dock_index + 1} "
                f"RED={red:.6f} GREEN={green:.6f} BLUE={blue:.6f} "
                f"WHITE={white:.6f} SYNC=0 TRANSMIT={transmit}")

    def _set_dock_led_color(self, dock_index, color):
        self._write_dock_colors({dock_index: color})

    def _with_strobe_channel(self, color, value):
        """Return `color` with the sense-pull channel set to `value` — plus
        the status channel when it is configured to mirror the sense pull.
        The supply gate is never touched here."""
        color = list(color)
        color[self._sense_index()] = float(value)
        status = self._status_index()
        if status is not None and self.argb_status_mode == 'sense':
            color[status] = float(value)
        return tuple(color)

    # --------------------------------------------------------------------
    # Dock supply (dark power management)
    # --------------------------------------------------------------------

    def _power_value(self, on):
        """LED value for the supply gate. Vortac docks cut the supply when
        the channel is HIGH, so `dock_power_invert` maps powered -> 0.0."""
        if self.dock_power_invert:
            return 0.0 if on else 1.0
        return 1.0 if on else 0.0

    def _policy_power(self, dock_name):
        """Where the supply of `dock_name` should be, derived from manager
        state alone. Fail-safe by construction: only a dock POSITIVELY known
        to be empty is de-energized, because a dark tool board means a lost
        mcu and a klippy shutdown."""
        if self._power_index() is None:
            return True
        if dock_name in self.dock_power_override:
            return self.dock_power_override[dock_name]
        if not self.dock_detection_valid:
            return True
        if dock_name in self.dock_uncertain:
            return True
        return self.dock_occupancy.get(dock_name) is not None

    def _status_value(self, dock_name, sense_on, power_on):
        if self.argb_status_mode == 'sense':
            return 1.0 if sense_on else 0.0
        if self.argb_status_mode == 'power':
            return 1.0 if power_on else 0.0
        # 'detected': full = tool found here, dim = confirmed empty,
        # off = no trustworthy map for this dock.
        if not self.dock_detection_valid or dock_name in self.dock_uncertain:
            return 0.0
        if self.dock_occupancy.get(dock_name) is not None:
            return 1.0
        return self.argb_status_dim

    def _apply_dock_leds(self, sense=None, strobe=False):
        """Reconcile every dock LED from manager state in one pass.

        `sense` optionally overrides the sense-pull channel per dock index
        (the detection strobe uses that); everything else is derived from
        occupancy, the supply overrides and the configured modes. Channels
        this module does not own are preserved as-is.

        `strobe` makes the status LED mirror the sense pull regardless of
        argb_status_mode, so a detection pass is always visible walking down
        the docks instead of flickering a half-rebuilt occupancy map."""
        if not self.argb_led:
            return
        sense = sense or {}
        colors = self._get_led_color_data()
        sense_idx = self._sense_index()
        status_idx = self._status_index()
        power_idx = self._power_index()
        out = {}
        for i in range(self.dock_count):
            dock = self._dock_name(i)
            sense_on = bool(sense.get(i, True))
            power_on = self._policy_power(dock)
            color = list(colors[i])
            color[sense_idx] = 1.0 if sense_on else 0.0
            if power_idx is not None:
                color[power_idx] = self._power_value(power_on)
            if status_idx is not None:
                color[status_idx] = (
                    (1.0 if sense_on else 0.0) if strobe
                    else self._status_value(dock, sense_on, power_on))
            out[i] = tuple(color)
        self._write_dock_colors(out)

    def _set_dock_power(self, dock_name, on, settle=True):
        """Override the supply of one dock and push it to the LED. Energizing
        waits `dock_power_settle` so the board is up before anything relies
        on it — but only when this actually changed the state, so repeated
        (idempotent) calls on the error paths do not stack up dwells."""
        was_on = self._policy_power(dock_name)
        self.dock_power_override[dock_name] = bool(on)
        self._apply_dock_leds()
        if on and not was_on and settle and self.dock_power_settle > 0.0:
            reactor = self.printer.get_reactor()
            reactor.pause(reactor.monotonic() + self.dock_power_settle)

    def _release_dock_power(self, dock_name):
        """Drop a transient override and fall back to the occupancy policy.

        Reconciles unconditionally: callers run this right after changing
        dock_occupancy, and the policy (plus the 'detected' status LED) has
        to follow that change even when no override was in play."""
        self.dock_power_override.pop(dock_name, None)
        self._apply_dock_leds()

    def _force_all_docks_powered(self, reason):
        """Energize every dock and keep it that way. Used before a restart:
        WS2812 latch across a klippy restart, and a dock left dead would
        starve its tool board before klippy:mcu_identify can find it."""
        if self._power_index() is None:
            return
        for i in range(self.dock_count):
            self.dock_power_override[self._dock_name(i)] = True
        try:
            self._apply_dock_leds()
        except Exception as e:
            logging.warning("vortac_manager: could not energize docks "
                            "before %s: %s", reason, e)
            return
        logging.info("vortac_manager: all docks energized before %s", reason)

    def _dock_power_map(self):
        return {self._dock_name(i): self._policy_power(self._dock_name(i))
                for i in range(self.dock_count)}

    # --------------------------------------------------------------------
    # Dock power watchdog
    # --------------------------------------------------------------------

    def _power_mismatches(self):
        """Dock indices whose supply channel does not match the policy.
        Reads the neopixel's host-side color_data — no mcu traffic."""
        if self._power_index() is None:
            return []
        colors = self._get_led_color_data()
        idx = self._power_index()
        return [i for i in range(self.dock_count)
                if abs(colors[i][idx]
                       - self._power_value(
                           self._policy_power(self._dock_name(i)))) > 1e-6]

    def _start_power_watchdog(self):
        if self._power_index() is None or self.dock_power_watchdog <= 0.0:
            return
        reactor = self.printer.get_reactor()
        reactor.register_timer(
            self._watchdog_timer_cb,
            reactor.monotonic() + self.dock_power_watchdog)
        logging.info(
            "vortac_manager: dock power watchdog armed (every %.1fs)",
            self.dock_power_watchdog)

    def _watchdog_timer_cb(self, eventtime):
        """Timer half: detect only — timers must not block. The fix runs as
        a gcode command through the queue, so it serializes behind any tool
        change or detection instead of interleaving with it (the handler
        recomputes under the mutex, so it never applies a stale view)."""
        reactor = self.printer.get_reactor()
        try:
            mismatched = self._power_mismatches()
        except Exception:
            # Broken LED lookup would spam this every period — say it once
            # and stand down; config validation should have caught it.
            logging.exception(
                "vortac_manager: dock power watchdog cannot read the LED "
                "state — watchdog disabled")
            return reactor.NEVER
        if mismatched and not self._watchdog_fix_pending:
            self._watchdog_fix_pending = True
            reactor.register_callback(self._watchdog_fix_cb)
        return eventtime + self.dock_power_watchdog

    def _watchdog_fix_cb(self, eventtime):
        try:
            gcode = self.printer.lookup_object('gcode')
            gcode.run_script('_VORTAC_DOCK_POWER_SYNC')
        except Exception:
            logging.exception("vortac_manager: dock power watchdog fix "
                              "failed")
        finally:
            self._watchdog_fix_pending = False

    def cmd_VORTAC_DOCK_POWER_SYNC(self, gcmd):
        """Re-assert the supply channel against the policy. Writes ONLY the
        power channel of mismatched docks — sense pull and status LED stay
        untouched, so a running VORTAC_DOCK_STROBE debug session is not
        disturbed. The re-check runs under the gcode mutex, i.e. against the
        CURRENT policy, never a view from before a queued tool change."""
        mismatched = self._power_mismatches()
        if not mismatched:
            return
        colors = self._get_led_color_data()
        idx = self._power_index()
        out = {}
        for i in mismatched:
            color = list(colors[i])
            color[idx] = self._power_value(
                self._policy_power(self._dock_name(i)))
            out[i] = tuple(color)
        self._write_dock_colors(out)
        names = ', '.join(self._dock_name(i) for i in mismatched)
        msg = (f"Vortac: dock supply channel on {names} did not match the "
               f"policy (manual SET_LED?) — restored. A tampered supply that "
               f"survives into a service restart strands the tool board, so "
               f"the watchdog reverts it.")
        gcmd.respond_info(msg)
        logging.warning(msg)

    def _report(self, gcmd, msg):
        if gcmd is not None:
            gcmd.respond_info(msg)
        else:
            logging.info(msg)

    def _handover_enabled(self):
        """Whether the dead-break half of the handover may run: it needs
        handover mode, a supply gate, and a dock whose sense pull survives
        losing that supply (see _power_off_before_backout)."""
        return (self.dock_power_mode == 'handover'
                and self._power_index() is not None
                and not self.sense_needs_dock_power)

    def _power_on_at_contact(self, tool, dock_name, gcmd):
        """Park: energize the dock once the pogos are mated.

        Called at the slid-in position, BEFORE the checked Z drop. The
        spring-loaded dock board rides along through the whole Z travel, so
        the pogos make and break contact on the Y slide only — which is why
        contact is verified here, while the dock is still dead, and the
        supply comes up before anything moves again.

        The two readings around the switch also self-diagnose: if dock_sense
        cannot see the mated contact while the dock is dead but can see it
        once it is live, the dock's sense pull hangs off the switched supply.
        That is recorded once and disables the dead-break on the fetch side,
        where it would otherwise fake a clean release."""
        if self._power_index() is None or self._policy_power(dock_name):
            return
        # SET_LED takes effect immediately, NOT in step with the move queue,
        # so the slide-in must be finished before the supply comes up —
        # otherwise the pogos would mate live. This wait is unconditional:
        # a tool without sense readings has no is_docked() to wait on, but
        # still has a queued G1.
        self._wait_sense()
        contact = tool.is_docked() if tool.sense_ready() else None
        self._set_dock_power(dock_name, True)
        if contact is None or contact:
            return
        self._wait_sense()
        if tool.is_docked():
            if not self.sense_needs_dock_power:
                self.sense_needs_dock_power = True
                self._report(
                    gcmd,
                    f"Vortac: {tool.tool_id} only reports dock contact at "
                    f"{dock_name} once the dock is energized — the dock's "
                    f"sense pull depends on the switched supply. Dead-break "
                    f"disabled for this session; dock changes keep the "
                    f"supply on while the pogos separate.")
        else:
            self._report(
                gcmd,
                f"Vortac warning: no dock contact for {tool.tool_id} at "
                f"{dock_name} after sliding in — the checked drop below "
                f"will retry, check dock calibration and pogo pins.")

    def _power_off_before_backout(self, tool, dock_name, gcmd):
        """Fetch: cut the dock supply after the checked lift and before the
        Y backout, so the pogos separate dead. The grabber already carries
        the tool, so the board keeps its supply from the other side.

        Guard: the backout check reads a clean release from dock_sense going
        HIGH. If the dock's sense pull died with the supply, dock_sense would
        go HIGH right here — with the tool not having moved at all — and
        every release check would silently pass. So verify dock_sense is
        still LOW; if not, restore the supply and stay powered from now on."""
        if not self._handover_enabled():
            return
        if self.dock_power_override.get(dock_name) is False:
            return          # already cut earlier in this fetch
        if not tool.sense_ready() or not tool.is_grabbed():
            # Without confirmed grab there is no second feed to rely on.
            return
        # The caller drains the queue on every checked lift step, but make the
        # invariant local: SET_LED is not synchronized with the move queue, so
        # no supply switch may happen with motion still pending.
        self._wait_sense()
        self._set_dock_power(dock_name, False)
        self._wait_sense()
        if tool.is_docked():
            return          # contact still reported — the cut is real
        self._set_dock_power(dock_name, True)
        self.sense_needs_dock_power = True
        self._report(
            gcmd,
            f"Vortac: dock_sense for {tool.tool_id} went HIGH the moment "
            f"{dock_name} was de-energized, while the tool had not moved — "
            f"the dock's sense pull hangs off the switched supply, so the "
            f"release check cannot be trusted without it. Supply restored, "
            f"dead-break disabled for this session.")
        self._wait_sense()

    def _sense_active_on_dock(self, color):
        """Only the sense-pull channel is load-bearing here — the status LED
        may legitimately be dark (see argb_status_mode)."""
        return color[self._sense_index()] > 0.0

    def _ensure_sense_active(self, gcmd=None):
        """Re-assert the full dock LED state: sense pull on everywhere, the
        supply per policy, the status LED per mode. Runs before every
        sense-dependent operation, so a manual SET_LED in between can
        neither disable the sense logic nor strand a tool board.
        Returns True if the sense pull had actually been off."""
        if not self.argb_led:
            return False
        colors = self._get_led_color_data()
        off = [self._dock_name(i) for i in range(self.dock_count)
               if not self._sense_active_on_dock(colors[i])]
        self._apply_dock_leds()
        if not off:
            return False
        self._dwell_for_sense()
        msg = (f"Vortac: sense pull ({self.argb_channel}) was off on "
               f"{', '.join(off)} — re-enabled before continuing")
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
        # Detection energizes every dock and reads sense pins — neither is
        # safe or meaningful with motion still queued (SET_LED takes effect
        # immediately, not in step with the move queue). Raises before any
        # state is touched, so an abort here leaves the previous map intact.
        self.printer.lookup_object('toolhead').wait_moves()
        all_docks = [self._dock_name(i) for i in range(self.dock_count)]
        detected = {dock: None for dock in all_docks}
        ambiguous = {}
        # Energize every dock for the whole pass, and drop any transient
        # handover override. A dock left dead cannot report the tool sitting
        # in it, so without this the detection would only ever confirm the
        # occupancy map it started from.
        self.dock_power_override = {dock: True for dock in all_docks}
        # Treat every dock as unresolved until this pass says otherwise: an
        # abort must not leave the previous map deciding what gets switched
        # off in the finally below.
        self.dock_uncertain = set(all_docks)
        try:
            # Detection baseline: sense pull off on all docks. Then each dock
            # is strobed on in turn; the docked tool flips its dock_sense.
            self._apply_dock_leds(
                sense={i: False for i in range(self.dock_count)},
                strobe=True)
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
                self._apply_dock_leds(
                    sense={i: (i == dock_index)
                           for i in range(self.dock_count)},
                    strobe=True)
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

                self._apply_dock_leds(
                    sense={i: False for i in range(self.dock_count)},
                    strobe=True)
                self._dwell_for_sense()

            self.dock_occupancy = detected
            self.dock_detection_valid = True
            if len(grabbed_ids) == 1:
                self.current_tool = self.tools[grabbed_ids[0]]
                self._assign_park_dock(self.current_tool, detected, gcmd)
            elif len(grabbed_ids) == 0:
                # A held tool without sense readings can never show up in
                # grabbed_ids — clearing current_tool on that non-evidence
                # would make the next change skip the park and drive the
                # held tool into the target's dock.
                if (self.current_tool is None
                        or self.current_tool.sense_ready()):
                    self.current_tool = None
            else:
                # The grabber holds at most one tool, so two grab_sense LOW
                # at once means at least one reading is lying. current_tool
                # is left alone, and the power decision below must not
                # trust a map built on lying sense pins.
                self._report(
                    gcmd,
                    f"Vortac warning: grab_sense reads LOW on "
                    f"{', '.join(sorted(grabbed_ids))} at once — physically "
                    f"impossible, check the grab sense wiring. Dock power "
                    f"stays on everywhere.")

            found_ids = set(
                tid for tid in detected.values() if tid is not None)
            found_ids.update(grabbed_ids)
            missing = [
                tid for tid in sorted(self.tools.keys())
                if tid not in found_ids and self.tools[tid].sense_ready()
            ]
            # Which docks may be de-energized afterwards. Ambiguous docks are
            # obviously untrustworthy; a tool this pass could not place makes
            # EVERY dock suspect, because it may well be sitting in a dock
            # that read empty precisely because its board was already dark.
            #
            # `missing` above is the user-facing list and deliberately only
            # names tools that DO deliver sense readings. The power decision
            # must be stricter: a tool with no readings at all (sense pins
            # absent but available: True) can never be located, and neither
            # can an unavailable tool whose mcu klippy still requires. Both
            # are permanently unaccounted for, so they pin every dock on.
            unaccountable = [
                tid for tid in sorted(self.tools.keys())
                if tid not in found_ids and not self.tools[tid].sense_ready()
            ]
            self.dock_uncertain = set(ambiguous)
            # A lying grab_sense (several tools "grabbed" at once) can hide
            # a parked tool from `missing` — it counts as found — so it
            # taints every dock as well.
            if (missing or unaccountable or self.unmanaged_tools
                    or len(grabbed_ids) > 1):
                self.dock_uncertain.update(all_docks)
            if unaccountable or self.unmanaged_tools:
                logging.info(
                    "vortac_manager: dock power pinned on — cannot locate %s",
                    ', '.join(unaccountable + self.unmanaged_tools))
            if gcmd is not None:
                self._respond_detection(gcmd, detected, grabbed_ids,
                                        missing, ambiguous, debug_lines)
            return detected
        finally:
            # Normal operating state: sense pull on everywhere, supply back
            # under the occupancy policy, status LED per mode. Never let a
            # failure here replace the error from the try body — the baseline
            # diagnostic is what the user needs to read.
            self.dock_power_override = {}
            try:
                self._apply_dock_leds()
                self._dwell_for_sense()
            except Exception:
                logging.exception(
                    "vortac_manager: could not restore the dock LEDs after "
                    "detection — the sense pull may still be off, which "
                    "invalidates dock_sense until the next tool change")

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
        # sense-pull channel holds the pad low. Manual LED tinkering may
        # have switched it off — reconcile every dock LED (sense pull,
        # supply, status) before any motion that relies on sense feedback.
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
            # park_dock/home_dock can be stale (a tool placed by hand, a
            # detection between fetch and park). Driving a held tool into an
            # occupied dock wrecks both tools, so refuse before moving.
            occupant = self.dock_occupancy.get(dock)
            if occupant is not None and occupant != tid:
                raise self.printer.command_error(
                    f"Vortac: refusing to park {tid} at {dock} — the dock map "
                    f"says {occupant} is already sitting there. Run "
                    f"VORTAC_DETECT to refresh the map, or free the dock.")
            # The dock-map check above cannot catch the case where we believe
            # we hold the very tool that is in fact still parked — after a
            # VORTAC_SET_CURRENT_TOOL on a docked tool, occupant == tid and
            # the park would drive the EMPTY grabber down onto it. dock_sense
            # settles that: a tool on the carriage reads HIGH, so a positive
            # LOW here is proof it never left its dock. Pin-less tools read
            # None and are not covered.
            if self.current_tool.is_docked():
                raise self.printer.command_error(
                    f"Vortac: refusing to park {tid} — dock_sense says it is "
                    f"still sitting in a dock, so the grabber is not actually "
                    f"holding it. Run VORTAC_DETECT to resync (the current "
                    f"tool was probably set manually).")
            self._park_at_dock(self.current_tool, dock, gcmd)
            self.dock_occupancy[dock] = tid
            # The dock now legitimately holds a tool, so the occupancy policy
            # keeps it energized on its own — drop the park override. Order
            # matters: releasing before the occupancy update would strand the
            # board we just parked.
            self.dock_uncertain.discard(dock)
            self._release_dock_power(dock)
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
            # Only the _dock_holding branch is evidence-based; the home_dock
            # fallback is a guess. Never fetch from a dock the map assigns to
            # a DIFFERENT tool — the clear-out below would erase that tool's
            # only occupancy record and leave its dock de-energized.
            occupant = self.dock_occupancy.get(dock)
            if occupant is not None and occupant != target.tool_id:
                raise self.printer.command_error(
                    f"Vortac: refusing to fetch {target.tool_id} from {dock} "
                    f"— the dock map says {occupant} is sitting there. Run "
                    f"VORTAC_DETECT to refresh the map.")
            # The grabber holds exactly one tool, and grab_sense outranks a
            # cleared current_tool (VORTAC_SET_CURRENT_TOOL CLEAR=1, or a
            # detection that could not see a sense-pin-less tool): fetching
            # with a tool still on the carriage rams it into the target's
            # dock. A legitimate swap never trips this — the park above
            # verifies grab went HIGH before it returns.
            grabbed = [tid for tid, t in sorted(self.tools.items())
                       if t.is_grabbed()]
            if grabbed:
                raise self.printer.command_error(
                    f"Vortac: refusing to fetch {target.tool_id} — "
                    f"grab_sense says {', '.join(grabbed)} is still in the "
                    f"grabber. Run VORTAC_DETECT (or park the held tool) "
                    f"first.")
            self._fetch_from_dock(target, dock, gcmd)
            self.dock_occupancy[dock] = None
            if target.sense_ready():
                # Positively empty (release and grab were verified), so the
                # policy de-energizes it by itself (the pogos are already
                # separated at this point — nothing switches under load
                # here, in handover mode the supply is off anyway).
                self.dock_uncertain.discard(dock)
            else:
                # Blind fetch: nothing verified the tool actually left the
                # dock — if the engage missed, its board is still sitting
                # there, and de-energizing it would take klippy down. Keep
                # the dock pinned on (detection can never resolve a
                # sense-pin-less tool either).
                self.dock_uncertain.add(dock)
            self._release_dock_power(dock)
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
        """Park a held tool: approach lifted, slide in, then hook in with a
        checked stepwise Z drop (mirror of the unhook verification — same
        step size and feedrate). dock_sense must be LOW at the hooked
        position before the grabber lets go; after backing out, the tool
        must still read docked and no longer grabbed.

        Dock supply: the dock is dead on arrival (it was empty), the pogos
        mate on the Y slide below, and the supply comes up right after that
        — before the Z drop, while the grabber still carries the tool. So
        the contact closes dead and nothing is switched under load."""
        x = tool.get_dock_pos(dock_name, 'x')
        y = tool.get_dock_pos(dock_name, 'y')
        z = tool.get_dock_pos(dock_name, 'z')
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(
            f'G90\n'
            f'G1 X{x} Y{y + DOCK_Y_SAFE} Z{z + DOCK_Z_CLEARANCE} '
            f'F{DOCK_APPROACH_F}\n'
            f'G1 Y{y} F{DOCK_SLIDE_F}')

        if not tool.sense_ready():
            # No sense feedback available — blind hook-in, but slow. The
            # supply still has to come up before the grabber lets go.
            if gcmd is not None:
                gcmd.respond_info(
                    f"Vortac: {tool.tool_id} sense pins not ready, "
                    f"hooking in blind")
            self._power_on_at_contact(tool, dock_name, gcmd)
            gcode.run_script_from_command(
                f'G1 Z{z} F{DOCK_UNHOOK_LIFT_F}')
            self.grabber.disengage(gcmd=gcmd)
            gcode.run_script_from_command(
                f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
            return

        self._ensure_sense_active(gcmd)
        # Pogos are mated by the Y slide above: verify contact with the dock
        # still dead, then energize before the checked drop starts.
        self._power_on_at_contact(tool, dock_name, gcmd)
        for attempt in range(1, DOCK_UNHOOK_RETRIES + 1):
            result = self._try_hook(tool, z)
            if result == 'seated':
                break
            if result == 'lost_grab':
                raise self.printer.command_error(
                    f"Vortac: grab_sense went HIGH while hooking "
                    f"{tool.tool_id} into {dock_name} — grabber lost the "
                    f"tool. Motion stopped; intervene manually.")
            # result == 'not_seated': no pogo contact at the hooked
            # position — misaligned on the screws. Lift back up and retry.
            if gcmd is not None:
                gcmd.respond_info(
                    f"Vortac: {tool.tool_id} shows no dock contact at the "
                    f"hooked position of {dock_name}, lifting to retry "
                    f"(attempt {attempt}/{DOCK_UNHOOK_RETRIES})")
            gcode.run_script_from_command(
                f'G1 Z{z + DOCK_Z_CLEARANCE} F{DOCK_UNHOOK_LIFT_F}')
        else:
            # Never seated — keep holding the tool, back out lifted. The
            # tool leaves with the grabber, so de-energize first and let the
            # pogos separate dead; no board is stranded, the grabber feeds it.
            # Wait first: SET_LED is not synchronized with the move queue.
            self._wait_sense()
            self._set_dock_power(dock_name, False)
            gcode.run_script_from_command(
                f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
            # Hand the dock back to the occupancy policy once the tool is
            # clear: a sticky False override would outlive this failure and
            # strand any tool that later reaches this dock outside the
            # manager (hand-placed, or a manual VORTAC_SET_CURRENT_TOOL
            # sequence).
            self._wait_sense()
            self._release_dock_power(dock_name)
            raise self.printer.command_error(
                f"Vortac: failed to seat {tool.tool_id} in {dock_name} "
                f"after {DOCK_UNHOOK_RETRIES} attempts (no dock contact); "
                f"tool is still held by the grabber, backed out lifted. "
                f"Check dock calibration and pogo pins.")

        self.grabber.disengage(gcmd=gcmd)
        gcode.run_script_from_command(
            f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
        self._wait_sense()
        self._log_unhook_checkpoint(tool, 'park-backout', DOCK_Y_SAFE)
        if not tool.is_docked():
            raise self.printer.command_error(
                f"Vortac: dock contact for {tool.tool_id} at {dock_name} "
                f"was lost while backing out after disengage — tool may "
                f"have been dragged out. Stopped; check the dock manually.")
        if tool.is_grabbed():
            raise self.printer.command_error(
                f"Vortac: {tool.tool_id} still reports grabbed after "
                f"disengage and backout at {dock_name}. Stopped; check "
                f"the grabber before continuing.")

    def _try_hook(self, tool, z):
        """One hook-in attempt: checked stepwise Z drop from clearance to
        the hooked position (same steps/feedrate as the unhook lift).
        grab_sense must STAY LOW the whole way; dock_sense is expected to
        flip to LOW as the pogos make contact near the hooked position.

        Returns 'seated' (dock contact at hooked Z), 'not_seated' (no dock
        contact after the full drop), or 'lost_grab'. Leaves the toolhead
        at the hooked position (or wherever grab was lost)."""
        gcode = self.printer.lookup_object('gcode')
        cur_z = z + DOCK_Z_CLEARANCE
        while cur_z > z + 1e-6:
            cur_z = max(cur_z - DOCK_UNHOOK_ZSTEP, z)
            gcode.run_script_from_command(
                f'G1 Z{cur_z:.3f} F{DOCK_UNHOOK_LIFT_F}')
            self._wait_sense()
            self._log_unhook_checkpoint(tool, 'drop', cur_z - z)
            if not tool.is_grabbed():
                return 'lost_grab'
        return 'seated' if tool.is_docked() else 'not_seated'

    def _fetch_from_dock(self, tool, dock_name, gcmd):
        """Fetch a docked tool: approach at saved hooked Z, slide in, engage
        the grabber, then unhook slowly with dock_sense verification and
        checked backward steps. On repeated unhook failure, reverse all
        steps (re-seat, disengage, back out empty) and raise.

        Dock supply: live on arrival (the tool is parked and awake). It is
        cut inside _try_unhook, after the checked lift and before the Y
        backout that separates the pogos — by then the grabber carries the
        tool and feeds it. Every failure exit re-energizes the dock, because
        a tool left behind in a dead dock would be a lost mcu at the next
        restart."""
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
            # No sense feedback available — blind unhook, but slow. No
            # dead-break either: without dock_sense there is no way to tell
            # a real release from a supply that just went away, so the pogos
            # separate live here (the dock is de-energized afterwards, once
            # the occupancy map says it is empty).
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

        # Cleared only once the tool has actually left the dock. Anything
        # else — lost grab, stuck hooks, a failed re-seat — means the tool
        # may still be sitting in this dock, so the supply has to come back.
        handed_off = False
        try:
            self._fetch_unhook_loop(tool, dock_name, y, z, gcmd)
            handed_off = True
        finally:
            if not handed_off:
                try:
                    # Deliberately no wait_moves() here: this runs while an
                    # exception is propagating, where the move queue may be in
                    # an error state, and energizing a dock that may still
                    # hold a board matters more than the arc-free window.
                    self._set_dock_power(dock_name, True)
                except Exception:
                    # Never let the recovery attempt mask the real failure —
                    # the original error is what the user has to read.
                    logging.exception(
                        "vortac_manager: could not re-energize %s after a "
                        "failed fetch", dock_name)

    def _fetch_unhook_loop(self, tool, dock_name, y, z, gcmd):
        """Unhook retry loop of _fetch_from_dock. Returns normally once the
        tool is out of the dock and confirmed grabbed; raises otherwise."""
        gcode = self.printer.lookup_object('gcode')
        for attempt in range(1, DOCK_UNHOOK_RETRIES + 1):
            result = self._try_unhook(tool, dock_name, y, z, gcmd)
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
        # release the grabber and back out empty. The supply has to be back
        # BEFORE the grabber lets go: from that moment the dock is the tool
        # board's only feed, and the caller's finally comes too late for that.
        # Wait for the last re-seat move first — SET_LED is not synchronized
        # with the move queue, so switching now would energize a contact that
        # is still closing.
        self._wait_sense()
        self._set_dock_power(dock_name, True)
        self.grabber.disengage(gcmd=gcmd)
        gcode.run_script_from_command(
            f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')
        raise self.printer.command_error(
            f"Vortac: failed to unhook {tool.tool_id} from {dock_name} "
            f"after {DOCK_UNHOOK_RETRIES} attempts; tool left in dock, "
            f"grabber disengaged")

    def _try_unhook(self, tool, dock_name, y, z, gcmd):
        """One unhook attempt from the hooked position.

        Stepwise slow lift, during which dock_sense must STAY LOW (the
        spring-loaded dock board rides along; losing pogo contact here
        means the tool is tilting on stuck hooks). Then checked backward
        steps, where dock_sense going HIGH is the clean release.

        Between the two phases the dock supply is cut (handover mode): the
        pogos only separate on the Y steps below, and by now the lift has
        confirmed both a held tool and intact dock contact — the two
        conditions that make it safe to hand the feed over to the grabber.

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
        self._power_off_before_backout(tool, dock_name, gcmd)
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
        if self._power_index() is None:
            power_line = f"disabled ({self.dock_power_mode})"
        else:
            power_line = f"{self.dock_power_mode}: " + ', '.join(
                f"{d}={'on' if on else 'OFF'}"
                for d, on in sorted(self._dock_power_map().items()))
            if self.dock_uncertain:
                power_line += (f"  [held on: "
                               f"{', '.join(sorted(self.dock_uncertain))}]")
            if self.sense_needs_dock_power:
                power_line += "  [dead-break disabled: sense needs dock power]"
        gcmd.respond_info(
            f"Vortac status:\n"
            f"  Current tool : {cur}\n"
            f"  Tools loaded : {tools_line}\n"
            f"  Dock map     : {occ or '(unknown)'}\n"
            f"  Detection    : {'valid' if self.dock_detection_valid else 'bootstrap'}\n"
            f"  Dock power   : {power_line}\n"
            f"  Status LED   : {self.argb_status_mode}\n"
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
            lines.append(
                f"  Dock sense pull ({self.argb_channel}): {states}")
            if any(not self._sense_active_on_dock(colors[i])
                   for i in range(self.dock_count)):
                lines.append(
                    f"  WARNING: dock_sense is only valid while a dock's "
                    f"sense pull ({self.argb_channel}) holds the pad low")
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
        mirrored = ('' if self.argb_status_mode != 'sense'
                    else f" (+{self.argb_status_channel}, mirroring)")
        gcmd.respond_info(
            f"Set {dock} sense pull ({self.argb_channel}){mirrored} "
            f"to {value:.3f}")

    def cmd_VORTAC_DOCK_POWER(self, gcmd):
        """Report or override the dock supply.

        VALUE=1 is always allowed. VALUE=0 on a dock that is believed to hold
        a tool needs FORCE=1, because de-energizing it drops that tool board
        off the CAN bus and takes klippy down — which is exactly what you
        want for isolating a bus fault or for flashing one board (ONLY=1),
        and never what you want by accident."""
        if self._power_index() is None:
            raise gcmd.error(
                f"Vortac: dock power management is disabled "
                f"(dock_power_mode: {self.dock_power_mode}, "
                f"dock_power_channel: {self.dock_power_channel or 'none'})")
        dock_arg = gcmd.get('DOCK', None)
        value = gcmd.get_int('VALUE', None, minval=0, maxval=1)
        only = gcmd.get_int('ONLY', 0, minval=0, maxval=1)
        force = gcmd.get_int('FORCE', 0, minval=0, maxval=1)
        if dock_arg is None and value is None:
            state = ', '.join(
                f"{d}={'on' if on else 'OFF'}"
                for d, on in sorted(self._dock_power_map().items()))
            gcmd.respond_info(
                f"Vortac dock power ({self.dock_power_mode}, channel "
                f"{self.dock_power_channel}"
                f"{', inverted' if self.dock_power_invert else ''}):\n"
                f"  {state}\n"
                f"  Sense pull needs dock power: "
                f"{'yes (dead-break disabled)' if self.sense_needs_dock_power else 'no'}")
            return
        if dock_arg is None:
            raise gcmd.error("Missing DOCK=dockN")
        dock = dock_arg.strip().lower()
        self._parse_dock_index(dock, gcmd)
        if value is None:
            raise gcmd.error("Missing VALUE=0|1")

        targets = {dock: bool(value)}
        if only:
            # Isolation mode: this dock as requested, every other dock the
            # opposite way. Used to make canbus_query/flash_can unambiguous.
            for i in range(self.dock_count):
                other = self._dock_name(i)
                if other != dock:
                    targets[other] = not value
        # "Believed empty" is only worth anything if the map is trustworthy:
        # without a valid detection, or on a dock the last pass could not
        # resolve, dock_occupancy may be a stale home_dock seed. Treat every
        # such dock as possibly occupied rather than possibly empty.
        risky = []
        for d, on in sorted(targets.items()):
            if on:
                continue
            if (self.dock_occupancy.get(d) is not None
                    or not self.dock_detection_valid
                    or d in self.dock_uncertain):
                risky.append(d)
        if risky and not force:
            raise gcmd.error(
                f"Vortac: refusing to de-energize {', '.join(risky)} — a tool "
                f"is parked there, or the dock map is not trustworthy enough "
                f"to say it is empty (run VORTAC_DETECT). Cutting the supply "
                f"of a parked board drops it off the CAN bus and shuts klippy "
                f"down. Add FORCE=1 if that is what you intend.")
        # A manual command can arrive with moves still queued; never switch a
        # supply while the toolhead is in motion near a dock.
        self.printer.lookup_object('toolhead').wait_moves()
        for d, on in sorted(targets.items()):
            self._set_dock_power(d, on)
        state = ', '.join(f"{d}={'on' if on else 'OFF'}"
                          for d, on in sorted(targets.items()))
        gcmd.respond_info(
            f"Vortac dock power override: {state}. Overrides are dropped by "
            f"the next VORTAC_DETECT; every dock is re-energized before a "
            f"RESTART.")

    def cmd_VORTAC_STATUS_LED(self, gcmd):
        modes = ('sense', 'power', 'detected')
        mode = gcmd.get('MODE', None)
        if mode is None:
            gcmd.respond_info(
                f"Vortac status LED ({self.argb_status_channel}): "
                f"{self.argb_status_mode} (available: {', '.join(modes)})")
            return
        mode = mode.strip().lower()
        if mode not in modes:
            raise gcmd.error(
                f"Unknown MODE '{mode}' (expected {', '.join(modes)})")
        self.argb_status_mode = mode
        self._apply_dock_leds()
        gcmd.respond_info(
            f"Vortac status LED now shows: {mode}. Set argb_status_mode in "
            f"[vortac_manager] to make this the default.")

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
            'park_dock': dict(self.park_dock),
            'dock_detection_valid': self.dock_detection_valid,
            'dock_uncertain': sorted(self.dock_uncertain),
            # dock_power is {} when management is off, so a dashboard can
            # tell "not managed" from "switched off".
            'dock_power': (self._dock_power_map()
                           if self._power_index() is not None else {}),
            'dock_power_mode': self.dock_power_mode,
            'argb_status_mode': self.argb_status_mode,
            'sense_needs_dock_power': self.sense_needs_dock_power,
            'calibration_dock': self.calibration_dock,
            'calibration_tool': (self.calibration_tool.tool_id
                                 if self.calibration_tool else None),
            'qgl_state': self.qgl_state.state if self.qgl_state else None,
        }


def load_config(config):
    return VortacManager(config)
