# vortac_tool.py — per-tool config sections for the Vortac toolchanger
#
# Each [vortac_tool <name>] block describes ONE logical tool: its target MCU
# namespace, gcode offsets, sense pins, and per-dock hooked/engage positions.
# Normally the section is INJECTED by include_with from the MCU template
# ([vortac_tool TN] placeholder -> [vortac_tool miniGrey]) with tool_index,
# mcu_name, canbus_uuid and extruder_name auto-filled; a statically written
# [vortac_tool T0] still works (identity derived from the trailing number).
#
# Hardware sections ([mcu <name>], extruder, fans, LEDs, etc.) must be present
# in normal Klipper config before this extra loads. Klipper registers MCU pin
# chips before extras are initialized, so [vortac_tool] intentionally does not
# inject [mcu ...] sections at runtime.
#
# GHOST sections: dock calibration is persisted via SAVE_CONFIG into this
# section. When a tool file is commented out, its `#*# [vortac_tool <name>]`
# autosave block remains and Klipper still instantiates a VortacTool from the
# params_*-only config. Such a ghost must load without error, defaults to
# available=False (no sense pins configured), and simply preserves the
# calibration data until the tool file is included again. A sense-pin-less
# tool that should still be usable (degraded blind-unhook mode) must set
# `available: True` explicitly.
#
# vortac_manager iterates registered VortacTool objects, only wires
# T<tool_index> commands for available tools, and uses get_dock_pos() to
# drive the dock-approach motion.


import logging
import re


def _trailing_int(name):
    m = re.search(r'(\d+)$', name)
    return int(m.group(1)) if m else None


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

        # Identity. Injected sections get tool_index/mcu_name/extruder_name
        # from include_with; a static [vortac_tool T1] derives them from its
        # trailing number. A ghost (autosave-only section, see module
        # docstring) has neither — tool_index stays None, which is fine as
        # long as the tool is not available.
        default_index = _trailing_int(self.tool_id)
        explicit_index = config.getint('tool_index', default=None, minval=0)
        # Explicit vs name-derived matters for the manager's contiguity
        # check: a ghost autosave section left over from the OLD naming
        # scheme ([vortac_tool T1]) derives an index from its name but must
        # not count against the live tools' index range.
        self.tool_index_explicit = explicit_index is not None
        self.tool_index = (explicit_index if explicit_index is not None
                           else default_index)
        default_mcu = (f"tool{self.tool_index}"
                       if self.tool_index is not None else None)
        self.mcu_name = config.get('mcu_name', default=default_mcu)
        self.canbus_uuid = config.get('canbus_uuid', default='').lower().strip()
        default_extruder = None
        if self.tool_index == 0:
            default_extruder = 'extruder'
        elif self.tool_index is not None:
            default_extruder = f"extruder{self.tool_index}"
        self.extruder_name = config.get(
            'extruder_name', default=default_extruder)

        # Tool-change config. home_dock is intentionally NOT derived from
        # tool_index — order-based numbering makes the index unstable while
        # the physical dock is not. Injected tools set it via include_with
        # overrides; static [vortac_tool T0] sections keep deriving it from
        # their (stable) trailing number.
        default_dock = (f"dock{default_index}"
                        if default_index is not None else None)
        self.home_dock = config.get('home_dock', default=default_dock)
        self.dock_sense_pin = config.get('dock_sense_pin', default=None)
        self.grab_sense_pin = config.get('grab_sense_pin', default=None)

        # Ghost detection: without sense pins this is (almost always) a
        # leftover autosave section for a commented-out tool — default to
        # unavailable so vortac_manager skips it while the calibration data
        # survives. Explicit `available: True` re-enables the degraded
        # blind-unhook mode for genuinely sense-pin-less tools.
        default_available = bool(self.dock_sense_pin and self.grab_sense_pin)
        self.available = config.getboolean('available', default_available)
        if not default_available and not self.available:
            logging.info(
                "vortac_tool %s: no sense pins configured — treating as "
                "ghost/unavailable; calibration data is preserved. Set "
                "'available: True' explicitly for a sense-pin-less tool.",
                self.tool_id)
        self.dock_sense_state = None
        self.grab_sense_state = None
        self.sense_state_time = None

        # GCode offsets (applied by manager when this tool is loaded)
        self.gcode_offset_x = config.getfloat('gcode_offset_x', 0.0)
        self.gcode_offset_y = config.getfloat('gcode_offset_y', 0.0)
        self.gcode_offset_z = config.getfloat('gcode_offset_z', 0.0)

        # Per-dock hooked/engage positions: params_<dock>_x|y|z
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
        """Persist hooked/engage XYZ for `dock_name` (user runs SAVE_CONFIG)."""
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
            'extruder_name': self.extruder_name,
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
