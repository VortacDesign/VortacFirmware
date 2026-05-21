# Klippy Extras

Custom Klipper modules for the Vortac toolchanger. `install.sh` symlinks every
`*.py` here into the running Klipper's `klippy/extras/` directory. After
`git pull` on the Pi, `systemctl restart klipper` picks up changes.

## Modules

### `vortac_grabber.py` — `[vortac_grabber]`

Hardware-only: AS5047D angle sensor (SPI), `manual_stepper grabber` closed-loop
control, LUT-based raw→true angle interpolation.

- **Gcode:** `VORTAC_CALIBRATE`, `VORTAC_SET_ZERO`, `VORTAC_MOVE`,
  `VORTAC_SIMPLE_READ`, `VORTAC_MESURE`.
- **Python API** (called by manager): `engage(angle=None, gcmd=None)`,
  `disengage(gcmd=None)`, `read_angle(gcmd=None)`.
- **Calibration:** synchronous direct-read (forward-only, multi-turn,
  last-turn-wins, circular mean per bin). The Klipper bulk-stream approach
  was lossy at sub-bit precision — direct reads are required for trustworthy
  LUTs.
- **Persistence:** `lookup_table` and `zero_pos_offset` written via
  `configfile.set` → user runs `SAVE_CONFIG`.

### `vortac_qgl_state.py` — `[vortac_qgl_state]`

Toggle the gantry between **frame-flat** (dock geometry valid) and
**bed-flat** (printing geometry valid) without re-probing.

- **Gcode:** `VORTAC_GANTRY_FLAT`, `VORTAC_GANTRY_TILT`, `VORTAC_QGL_STATUS`.
- **Mechanic:** at `klippy:ready`, hooks `quad_gantry_level.z_helper.adjust_steppers`
  to capture each delta vector into `stored_deltas`. FLAT replays
  `-stored_deltas`, TILT replays `+stored_deltas`. Wraps `QUAD_GANTRY_LEVEL`
  to zero `stored_deltas` when re-running while flat (so the new measurement
  starts clean).
- **Reset:** `homing:home_rails_end` on any `stepper_z*` clears state and
  sets state to `flat`.
- **State is RAM only** — power cycle, mid-print loss, or a fresh Z-home
  invalidates it; user must re-run QGL.
- **Manager dependency:** `VORTAC_DOCK_SAVE_POS` refuses unless `state == 'flat'`.

### `include_with.py` — `[include_with <namespace> <filename>]` + programmatic API

Load one Klipper template under a remapped MCU namespace; auto-rename sections
per `tool_index` so one PCB-level config can be instantiated N times.

- **Section rename rules** (only triggered for `tool_index ≥ 1`; the MCU rename
  is always applied):
  | Original | Tool 1 | Tool 2 |
  |---|---|---|
  | `[mcu <from>]` | `[mcu <to>]` | `[mcu <to>]` |
  | `[extruder]` | `[extruder1]` | `[extruder2]` |
  | `[tmcXXXX extruder]` | `[tmcXXXX extruder1]` | `[tmcXXXX extruder2]` |
  | `[fan]` | `[fan_generic tool1_fan]` | `[fan_generic tool2_fan]` |
  | `[heater_fan name]` | `[heater_fan tool1_name]` | `[heater_fan tool2_name]` |
  | `[neopixel name]` | `[neopixel tool1_name]` | `[neopixel tool2_name]` |
  | `[adxl345 (name)]` | `[adxl345 tool1(_name)]` | `[adxl345 tool2(_name)]` |
- **Value rewrite:** any `<from>:` pin token in option values becomes `<to>:`;
  whole-string section refs (e.g. `heater: extruder` → `heater: extruder1`)
  are also fixed up.
- **Skip-list** for `tool_index ≥ 1`: `resonance_tester`, `input_shaper`,
  `shaketune` (Klipper singletons that can't be renamed). Config sections can
  also pass `skip_sections:` as a comma/newline list; tool configs use this to
  skip the template `[mcu EBBCan]` because `[mcu toolN]` must be declared
  statically.
- **Overrides:** JSON `{"orig section": {"key": "value"}}` via `overrides:`
  config option, or programmatic `overrides=` arg.
- **Programmatic API** (used by `vortac_tool`):
  `include_with_remap(printer, parent_config, filepath, namespace,
  mcu_from=None, tool_index=0, overrides=None, skip_sections=None)`.

### `vortac_tool.py` — `[vortac_tool Tn]`

Per-tool logical definition. The tool hardware must already be present in
normal Klipper config (`[mcu toolN]` plus `[include_with toolN ...]`) before
this extra loads; Klipper registers MCU pin chips before extras run.

- **Availability:** manual `available:` flag, default `True`. For now, comment
  out absent tool includes in `tools.cfg` rather than relying on runtime CAN
  auto-detection.
- **Template loading:** lives in the tool config via `[include_with ...]`, not
  in `vortac_tool.py`.
- **Per-dock storage:** `params_<dock>_<x|y|z>`. Manager API:
  `get_dock_pos(dock, axis)`, `has_dock_pos(dock)`,
  `save_dock_pos(dock, x, y, z)`.
- **Templates:** `tool_activate_gcode`, `tool_deactivate_gcode` — run by
  manager around the dock motion. Jinja context: `tool` (this VortacTool),
  `dock=None`, `params`.

### `vortac_manager.py` — `[vortac_manager]`

Coordinator. At `klippy:connect` it discovers `[vortac_tool *]` (only those
with `available=True`), the grabber (required), and `[vortac_qgl_state]`
(optional). Registers `Tn` commands for every available tool.

- **Gcode:** `T0/T1/…` (registered dynamically), `VORTAC_STATUS`,
  `VORTAC_LOAD TOOL=Tn`, `VORTAC_UNLOAD`, `VORTAC_SET_CURRENT_TOOL`,
  `VORTAC_DETECT`, `VORTAC_SELECT_DOCK DOCK=dockN`,
  `VORTAC_DOCK_CAL_STATUS`, `VORTAC_DOCK_CAL_SAVE`,
  `VORTAC_DOCK_SAVE_POS TOOL=Tn DOCK=name`.
- **Tool change sequence** in `_change_to`:
  `tool_deactivate_gcode` → `VORTAC_GANTRY_FLAT` → `_park_at_dock(current)`
  → `_fetch_from_dock(target)` → `VORTAC_GANTRY_TILT` → `SET_GCODE_OFFSET`
  → `tool_activate_gcode`.
- **Dock motion** is hardcoded (constants at top of file):
  `DOCK_Y_SAFE = 50.0`, `DOCK_Z_CLEARANCE = 4.4`, `DOCK_APPROACH_F = 2000`,
  `DOCK_SLIDE_F = 500`. Edit there for global geometry tweaks; per-tool
  tweaks belong in `tool_activate_gcode` / `tool_deactivate_gcode`.
- **Detection:** `VORTAC_DETECT` turns all dock Tool_id channels on, strobes
  one dock's configured LED channel off at a time, reads each tool's cached
  `dock_sense_pin`, and restores every LED's original RGBW value afterward.
  The grabbed tool is detected from `grab_sense_pin`.
- **Dock calibration context:** `VORTAC_SELECT_DOCK DOCK=dockN` runs detection
  and stores the detected dock/tool pair. `VORTAC_DOCK_CAL_SAVE` saves the
  current XYZ to that selected tool/dock and fails if no tool was detected.

## Lifecycle & dependencies

```
config parse:
  [vortac_grabber]    → register grabber gcode commands
  [vortac_qgl_state]  → register FLAT/TILT (hooks attach at klippy:ready)
  [mcu toolN]         → register tool MCU pin chip early
  [include_with ...]  → inject remapped toolboard sections that use toolN pins
  [vortac_tool Tn]    → load logical dock/offset/tool metadata
  [vortac_manager]    → register VORTAC_LOAD/UNLOAD/STATUS/DOCK_SAVE_POS;
                        defer rest to klippy:connect

klippy:ready    → vortac_qgl_state hooks QGL.adjust_steppers + QUAD_GANTRY_LEVEL
klippy:connect  → vortac_manager finds tools/grabber/qgl, registers Tn commands
```

Hard dependencies:
- `vortac_manager` requires `[vortac_grabber]`.
- Each active tool requires a static `[mcu toolN]` section before `[include_with]`.
- `vortac_qgl_state` requires `[quad_gantry_level]` to be defined.

For config layout and the user-side workflow, see
[`configs/README.md`](../../configs/README.md).
