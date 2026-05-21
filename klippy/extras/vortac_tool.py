# vortac_tool.py — per-tool config sections for the Vortac toolchanger
#
# Each [vortac_tool Tn] block describes ONE logical tool: its target MCU
# namespace, gcode offsets, sense pins, and per-dock positions.
#
# Hardware sections ([mcu toolN], extruder, fans, LEDs, etc.) must be present
# in normal Klipper config before this extra loads. Klipper registers MCU pin
# chips before extras are initialized, so [vortac_tool] intentionally does not
# inject [mcu ...] sections at runtime.
#
# Phase 5's vortac_manager iterates registered VortacTool objects, only
# wires Tn commands for available tools, and uses get_dock_pos() to drive
# the dock-approach gcode template.


def _parse_dock_positions(config):
    """Parse params_<dock>_<axis> options into {dock_name: {x, y, z}}.

    Tolerates non-float legacy params_* entries (like the old
    params_x_doc_pos) by skipping them silently.
    """
    out = {}
    for opt in config.get_prefix_options('params_'):
        rest = opt[len('params_'):]
        if '_' not in rest:
            continue
        dock_part, axis = rest.rsplit('_', 1)
        if axis not in ('x', 'y', 'z'):
            continue
        try:
            val = config.getfloat(opt)
        except Exception:
            continue
        out.setdefault(dock_part, {})[axis] = val
    return out


class VortacTool:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()  # e.g. "vortac_tool T0"
        parts = self.name.split()
        self.tool_id = parts[1] if len(parts) >= 2 else self.name

        # Identity
        self.tool_index = config.getint('tool_index', minval=0)
        self.mcu_name = config.get('mcu_name')
        self.canbus_uuid = config.get('canbus_uuid', default='').lower().strip()
        self.available = config.getboolean('available', True)

        # Tool-change config
        self.home_dock = config.get('home_dock', default=None)
        self.dock_sense_pin = config.get('dock_sense_pin', default=None)
        self.grab_sense_pin = config.get('grab_sense_pin', default=None)
        self.dock_sense_state = None
        self.grab_sense_state = None
        self.sense_state_time = None

        # GCode offsets (applied by manager when this tool is loaded)
        self.gcode_offset_x = config.getfloat('gcode_offset_x', 0.0)
        self.gcode_offset_y = config.getfloat('gcode_offset_y', 0.0)
        self.gcode_offset_z = config.getfloat('gcode_offset_z', 0.0)

        # Per-dock positions: params_<dock>_x|y|z
        self.dock_positions = _parse_dock_positions(config)

        # Activate/deactivate gcode templates
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.tool_activate_gcode = gcode_macro.load_template(
            config, 'tool_activate_gcode', '')
        self.tool_deactivate_gcode = gcode_macro.load_template(
            config, 'tool_deactivate_gcode', '')

        if self.available and self.dock_sense_pin and self.grab_sense_pin:
            buttons = self.printer.load_object(config, 'buttons')
            buttons.register_buttons(
                [self.dock_sense_pin, self.grab_sense_pin],
                self._handle_sense_state)

    # --------------------------------------------------------------------
    # Python API for vortac_manager
    # --------------------------------------------------------------------

    def _handle_sense_state(self, eventtime, state):
        # buttons.register_buttons returns a bit mask in pin-list order.
        self.dock_sense_state = bool(state & 0x01)
        self.grab_sense_state = bool(state & 0x02)
        self.sense_state_time = eventtime

    def is_docked(self):
        # Sense lines are default HIGH and actively pulled LOW.
        return self.dock_sense_state is False

    def is_grabbed(self):
        return self.grab_sense_state is False

    def sense_ready(self):
        return (self.dock_sense_state is not None
                and self.grab_sense_state is not None)

    def get_dock_pos(self, dock_name, axis):
        """Return calibrated coordinate for this tool at `dock_name`.
        Raises on missing entry — manager surfaces this as a gcode error."""
        d = self.dock_positions.get(dock_name)
        if d is None or axis not in d:
            raise self.printer.command_error(
                f"vortac_tool {self.tool_id}: no '{axis}' position saved for "
                f"dock '{dock_name}' (option params_{dock_name}_{axis})")
        return d[axis]

    def has_dock_pos(self, dock_name):
        d = self.dock_positions.get(dock_name)
        return d is not None and all(a in d for a in ('x', 'y', 'z'))

    def save_dock_pos(self, dock_name, x, y, z):
        """Persist (x, y, z) for `dock_name` into config (user runs SAVE_CONFIG)."""
        cfg = self.printer.lookup_object('configfile')
        for axis, val in (('x', x), ('y', y), ('z', z)):
            cfg.set(self.name, f'params_{dock_name}_{axis}', f'{val:.4f}')
        self.dock_positions.setdefault(dock_name, {}).update(
            {'x': float(x), 'y': float(y), 'z': float(z)})

    # --------------------------------------------------------------------
    # Status (for Mainsail/Fluidd dashboards and templates)
    # --------------------------------------------------------------------

    def get_status(self, eventtime):
        return {
            'tool_id': self.tool_id,
            'tool_index': self.tool_index,
            'mcu_name': self.mcu_name,
            'canbus_uuid': self.canbus_uuid,
            'available': self.available,
            'home_dock': self.home_dock,
            'gcode_offset_x': self.gcode_offset_x,
            'gcode_offset_y': self.gcode_offset_y,
            'gcode_offset_z': self.gcode_offset_z,
            'dock_positions': self.dock_positions,
            'dock_sense_state': self.dock_sense_state,
            'grab_sense_state': self.grab_sense_state,
        }


def load_config_prefix(config):
    return VortacTool(config)
