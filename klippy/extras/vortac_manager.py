# vortac_manager.py — coordinator for the Vortac toolchanger.
#
# At klippy:connect this module discovers all [vortac_tool Tn] sections,
# the [vortac_grabber] hardware controller, and the optional
# [vortac_qgl_state] toggle, then registers Tn commands for the tools
# whose CAN UUIDs were detected as present.
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
# Phase 6 will add VORTAC_DETECT (ARGB strobe-by-subtraction) to populate
# the dock occupancy map at runtime instead of relying on the home_dock
# bootstrap.

import logging


# Hook-on-screws dock geometry. Approach happens at the saved (x, y, z) seat
# position, offset by Y_SAFE in Y for clearance and Z_CLEARANCE in Z for the
# engage/disengage drop. These match the original tool_doc_load / unload
# sequences; tweak here if the dock geometry changes.
DOCK_Y_SAFE      = 50.0    # mm, Y clearance for approach/depart
DOCK_Z_CLEARANCE = 4.4     # mm, Z drop during engage/disengage
DOCK_APPROACH_F  = 2000    # mm/min, fast move to dock front
DOCK_SLIDE_F     = 500     # mm/min, slow slide-in/out and Z hop


class VortacManager:
    def __init__(self, config):
        self.printer = config.get_printer()

        self.dock_count = config.getint('dock_count', minval=1)
        self.argb_led = config.get('argb_led', default=None)
        self.argb_channel = config.get('argb_channel', default='green')

        # gcode_macro is needed by [vortac_tool Tn]'s activate/deactivate
        # templates; load it eagerly so the order doesn't matter.
        self.printer.load_object(config, 'gcode_macro')

        # State (populated at klippy:connect)
        self.tools = {}             # tool_id -> VortacTool
        self.current_tool = None    # VortacTool or None
        self.dock_occupancy = {}    # dock_name -> tool_id or None
        self.grabber = None
        self.qgl_state = None

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'VORTAC_STATUS', self.cmd_VORTAC_STATUS,
            desc="Report current tool, dock map, QGL state")
        gcode.register_command(
            'VORTAC_LOAD', self.cmd_VORTAC_LOAD,
            desc="Manually load tool TOOL=Tn (no current tool)")
        gcode.register_command(
            'VORTAC_UNLOAD', self.cmd_VORTAC_UNLOAD,
            desc="Manually park the held tool back at its dock")
        gcode.register_command(
            'VORTAC_DOCK_SAVE_POS', self.cmd_VORTAC_DOCK_SAVE_POS,
            desc="Save current XYZ as held tool's position for DOCK=name")

        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)

    # --------------------------------------------------------------------
    # Connect-time discovery
    # --------------------------------------------------------------------

    def _handle_connect(self):
        gcode = self.printer.lookup_object('gcode')

        for name, obj in self.printer.objects.items():
            if not name.startswith('vortac_tool '):
                continue
            if not getattr(obj, 'available', False):
                logging.info(
                    "vortac_manager: skipping %s (not available)", name)
                continue
            self.tools[obj.tool_id] = obj
            if obj.home_dock:
                self.dock_occupancy.setdefault(obj.home_dock, obj.tool_id)

        self.grabber = self.printer.lookup_object('vortac_grabber', None)
        if self.grabber is None:
            raise self.printer.config_error(
                "vortac_manager: [vortac_grabber] is required but not loaded")

        self.qgl_state = self.printer.lookup_object('vortac_qgl_state', None)

        for tool_id, tool in self.tools.items():
            # Default-arg trick to capture tool_id per-iteration in the closure.
            gcode.register_command(
                tool_id,
                (lambda gcmd, tid=tool_id: self._cmd_change(gcmd, tid)),
                desc=f"Switch to {tool_id}")

        logging.info(
            "vortac_manager: registered tools=%s, grabber=%s, qgl_state=%s",
            sorted(self.tools.keys()), bool(self.grabber), bool(self.qgl_state))

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

        # 1. Deactivate current tool
        if self.current_tool is not None:
            self._render(self.current_tool.tool_deactivate_gcode,
                         tool=self.current_tool, dock=None, gcmd=gcmd)

        # 2. Frame-flat geometry for dock approach
        if self.qgl_state is not None:
            gcode.run_script_from_command('VORTAC_GANTRY_FLAT')

        # 3. Park the held tool (if any) at its current dock
        if self.current_tool is not None:
            dock = self._dock_holding(self.current_tool.tool_id) \
                   or self.current_tool.home_dock
            self._park_at_dock(self.current_tool, dock, gcmd)
            self.dock_occupancy[dock] = self.current_tool.tool_id

        # 4. Pick up the target tool from its current dock
        if target is not None:
            dock = self._dock_holding(target.tool_id) or target.home_dock
            self._fetch_from_dock(target, dock, gcmd)
            self.dock_occupancy[dock] = None

        self.current_tool = target

        # 5. Bed-flat geometry for printing
        if self.qgl_state is not None:
            gcode.run_script_from_command('VORTAC_GANTRY_TILT')

        # 6. Apply target offsets, run activate gcode
        if target is not None:
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
        """Park a held tool: approach at seat Z, slide in, drop onto dock
        screws, disengage the grabber, slide back out."""
        x = tool.get_dock_pos(dock_name, 'x')
        y = tool.get_dock_pos(dock_name, 'y')
        z = tool.get_dock_pos(dock_name, 'z')
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(
            f'G90\n'
            f'G1 X{x} Y{y + DOCK_Y_SAFE} Z{z} F{DOCK_APPROACH_F}\n'
            f'G1 Y{y} F{DOCK_SLIDE_F}\n'
            f'G1 Z{z - DOCK_Z_CLEARANCE} F{DOCK_SLIDE_F}')
        self.grabber.disengage(gcmd=gcmd)
        gcode.run_script_from_command(
            f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')

    def _fetch_from_dock(self, tool, dock_name, gcmd):
        """Fetch a docked tool: approach below seat so the grabber slides
        under the tool, slide in, engage the grabber, lift off the dock
        screws, slide back out."""
        x = tool.get_dock_pos(dock_name, 'x')
        y = tool.get_dock_pos(dock_name, 'y')
        z = tool.get_dock_pos(dock_name, 'z')
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(
            f'G90\n'
            f'G1 X{x} Y{y + DOCK_Y_SAFE} Z{z - DOCK_Z_CLEARANCE} '
            f'F{DOCK_APPROACH_F}\n'
            f'G1 Y{y} F{DOCK_SLIDE_F}')
        self.grabber.engage(gcmd=gcmd)
        gcode.run_script_from_command(
            f'G1 Z{z} F{DOCK_SLIDE_F}\n'
            f'G1 Y{y + DOCK_Y_SAFE} F{DOCK_SLIDE_F}')

    def _dock_holding(self, tool_id):
        for dock, tid in self.dock_occupancy.items():
            if tid == tool_id:
                return dock
        return None

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
        tools_line = (', '.join(sorted(self.tools.keys()))
                      if self.tools else '(none registered)')
        occ = ', '.join(
            f"{d}={tid or 'empty'}"
            for d, tid in sorted(self.dock_occupancy.items()))
        gcmd.respond_info(
            f"Vortac status:\n"
            f"  Current tool : {cur}\n"
            f"  Tools loaded : {tools_line}\n"
            f"  Dock map     : {occ or '(unknown)'}\n"
            f"  QGL state    : {qgl}")

    def cmd_VORTAC_LOAD(self, gcmd):
        target_id = gcmd.get('TOOL').strip()
        if not target_id.upper().startswith('T'):
            target_id = 'T' + target_id
        if target_id not in self.tools:
            raise gcmd.error(
                f"Tool {target_id} not registered or unavailable")
        if self.current_tool is not None:
            raise gcmd.error(
                f"Already holding {self.current_tool.tool_id}; "
                f"unload first or use {target_id} to swap directly")
        self._change_to(self.tools[target_id], gcmd)

    def cmd_VORTAC_UNLOAD(self, gcmd):
        if self.current_tool is None:
            raise gcmd.error("No tool currently held")
        self._change_to(None, gcmd)

    def cmd_VORTAC_DOCK_SAVE_POS(self, gcmd):
        dock = gcmd.get('DOCK').strip()
        if self.qgl_state is not None and self.qgl_state.state != 'flat':
            raise gcmd.error(
                f"Refuse to save dock position: gantry is "
                f"{self.qgl_state.state!r}; run VORTAC_GANTRY_FLAT first "
                f"(dock geometry is only valid frame-flat)")
        if self.current_tool is None:
            raise gcmd.error(
                "No tool currently grabbed; cannot save dock position")
        toolhead = self.printer.lookup_object('toolhead')
        x, y, z = toolhead.get_position()[:3]
        self.current_tool.save_dock_pos(dock, x, y, z)
        gcmd.respond_info(
            f"Saved {self.current_tool.tool_id} @ {dock}: "
            f"X={x:.4f} Y={y:.4f} Z={z:.4f}.  Run SAVE_CONFIG to persist.")

    # --------------------------------------------------------------------
    # Status (for templates / dashboards)
    # --------------------------------------------------------------------

    def get_status(self, eventtime):
        return {
            'current_tool': (self.current_tool.tool_id
                             if self.current_tool else None),
            'tools': sorted(self.tools.keys()),
            'dock_occupancy': dict(self.dock_occupancy),
            'qgl_state': self.qgl_state.state if self.qgl_state else None,
        }


def load_config(config):
    return VortacManager(config)
