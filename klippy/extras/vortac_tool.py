# vortac_tool.py — per-tool config sections for the Vortac toolchanger
#
# Each [vortac_tool Tn] block describes ONE tool: its CAN UUID, its target
# MCU namespace, the toolboard template to load, gcode offsets, sense pins,
# and per-dock positions. On startup:
#
#   1. Optionally scans the CAN bus (delegates to klipper/scripts/canbus_query.py)
#      to confirm the tool's UUID is actually present.
#   2. If present (or auto_detect is False): calls include_with_remap to
#      inject the toolboard template under the per-tool MCU namespace, with
#      the per-tool canbus_uuid as an override so multiple tools sharing
#      one template each get their own UUID.
#   3. If absent: sets self.available = False and skips template injection.
#      Klipper never tries to connect to a missing MCU.
#
# Phase 5's vortac_manager iterates registered VortacTool objects, only
# wires Tn commands for available tools, and uses get_dock_pos() to drive
# the dock-approach gcode template.

import os
import logging
import subprocess


# Cache the CAN scan result across all [vortac_tool] instances so we only
# query the bus once per Klipper startup.
_CAN_SCAN_CACHE = {}  # {interface: set_of_uuid_hex_strings_or_None}


def _scan_can_uuids(interface, timeout_s, klipper_root):
    """Run klipper/scripts/canbus_query.py and return a set of UUID hex
    strings. Returns None on any failure — callers should treat None as
    "scan unavailable, assume present" so the absence of canbus_query
    doesn't silently drop every tool.
    """
    if klipper_root is None:
        klipper_root = os.path.expanduser('~/klipper')
    script = os.path.join(klipper_root, 'scripts', 'canbus_query.py')
    if not os.path.exists(script):
        logging.warning(
            "vortac_tool: canbus_query.py not found at %s; CAN auto-detect "
            "disabled (set 'klipper_root' on [vortac_tool] or 'auto_detect: "
            "False' to silence)", script)
        return None
    try:
        out = subprocess.check_output(
            ['python3', script, interface],
            stderr=subprocess.STDOUT, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        logging.warning(
            "vortac_tool: canbus_query timed out on %r", interface)
        return None
    except subprocess.CalledProcessError as e:
        logging.warning(
            "vortac_tool: canbus_query failed (exit %d): %s",
            e.returncode,
            e.output.decode('ascii', errors='ignore').strip())
        return None
    except Exception as e:
        logging.warning("vortac_tool: canbus_query error: %s", e)
        return None

    uuids = set()
    text = out.decode('ascii', errors='ignore')
    for line in text.splitlines():
        # Klipper emits lines like:
        #   "Found canbus_uuid=AABBCCDDEEFF, Application: Klipper"
        if 'canbus_uuid=' in line:
            tail = line.split('canbus_uuid=', 1)[1]
            uuid_hex = tail.split(',', 1)[0].strip().lower()
            if uuid_hex:
                uuids.add(uuid_hex)
    return uuids


def _get_can_uuids(interface, timeout_s, klipper_root):
    if interface not in _CAN_SCAN_CACHE:
        _CAN_SCAN_CACHE[interface] = _scan_can_uuids(
            interface, timeout_s, klipper_root)
    return _CAN_SCAN_CACHE[interface]


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
        self.canbus_uuid = config.get('canbus_uuid').lower().strip()
        self.mcu_name = config.get('mcu_name')

        # Template
        self.config_template = config.get('config_template')

        # Tool-change config
        self.home_dock = config.get('home_dock', default=None)
        self.dock_sense_pin = config.get('dock_sense_pin', default=None)
        self.grab_sense_pin = config.get('grab_sense_pin', default=None)

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

        # CAN auto-detect
        self.auto_detect = config.getboolean('auto_detect', True)
        self.can_interface = config.get('can_interface', default='can0')
        self.can_scan_timeout = config.getfloat(
            'can_scan_timeout', default=2.0, above=0.0)
        self.klipper_root = config.get('klipper_root', default=None)

        # Probe CAN, decide availability, inject template if present.
        self.available = self._determine_availability()
        if self.available:
            self._inject_template(config)
        else:
            logging.info(
                "vortac_tool %s: UUID %s not found on %s; skipping template "
                "injection (tool marked unavailable)",
                self.tool_id, self.canbus_uuid, self.can_interface)

    # --------------------------------------------------------------------
    # Setup helpers
    # --------------------------------------------------------------------

    def _determine_availability(self):
        if not self.auto_detect:
            return True
        uuids = _get_can_uuids(
            self.can_interface, self.can_scan_timeout, self.klipper_root)
        if uuids is None:
            # Scan unavailable — fail open so missing canbus_query.py
            # doesn't silently drop every tool.
            logging.warning(
                "vortac_tool %s: CAN scan unavailable; assuming present",
                self.tool_id)
            return True
        return self.canbus_uuid in uuids

    def _inject_template(self, parent_config):
        # Klipper places extras/ on sys.path during module load, so a flat
        # `import include_with` works. (The package form `from extras import
        # include_with` is also valid; flat keeps us robust to either.)
        try:
            from extras import include_with as iw
        except ImportError:
            import include_with as iw

        # Override the template's canbus_uuid so multiple tools sharing one
        # template each get their own UUID. mcu_from is auto-detected, so we
        # don't know the original MCU section name yet; let include_with_remap
        # discover it and we apply the override on whatever it returns.
        # Strategy: read the template once to find mcu_from, then build the
        # override dict keyed on the original section name.
        printer_config = self.printer.lookup_object('configfile')
        template_path = iw._resolve_filepath(self.printer, self.config_template)
        template = printer_config.read_config(template_path)
        mcu_from = iw._autodetect_mcu_from(template)
        if mcu_from is None:
            raise parent_config.error(
                f"vortac_tool {self.tool_id}: template '{self.config_template}' "
                f"has no [mcu <name>] section")

        overrides = {
            f'mcu {mcu_from}': {'canbus_uuid': self.canbus_uuid},
        }

        iw.include_with_remap(
            printer=self.printer,
            parent_config=parent_config,
            filepath=self.config_template,
            namespace=self.mcu_name,
            mcu_from=mcu_from,
            tool_index=self.tool_index,
            overrides=overrides,
        )

    # --------------------------------------------------------------------
    # Python API for vortac_manager
    # --------------------------------------------------------------------

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
        }


def load_config_prefix(config):
    return VortacTool(config)
