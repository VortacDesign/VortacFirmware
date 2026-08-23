# Klippy Extras

Custom Klipper modules for the Vortac toolchanger. `install.sh` symlinks every
`*.py` here into the running Klipper's `klippy/extras/` directory. After
`git pull` on the Pi, `systemctl restart klipper` picks up changes.

## Modules

### `vortac_grabber.py` — `[vortac_grabber]`

Hardware-only: AS5047D angle sensor (SPI), `manual_stepper grabber` closed-loop
control, LUT-based raw→true angle interpolation.

- **Python API** (called by manager): `engage(angle=None, gcmd=None)`,
  `disengage(gcmd=None)`, `read_angle(gcmd=None)`.
- **Calibration** (`VORTAC_CALIBRATE`): synchronous direct-read (multi-turn,
  last-turn-wins, circular mean per bin). `DIR=both` (default) sweeps cw
  then ccw, stores the per-bin circular midpoint and reports the cw/ccw
  spread as a backlash measurement; `DIR=cw|ccw` sweeps one direction only
  (use the later operating/approach direction). The Klipper bulk-stream
  approach was lossy at sub-bit precision — direct reads are required for
  trustworthy LUTs.
- **Closed-loop moves** (`VORTAC_MOVE`, engage/disengage): each pass
  commands the full remaining error in one `manual_move`, re-measures,
  repeats (typically 1–2 moves; lost steps show up in the next absolute
  reading). `MODE=cw|ccw` constrained moves stop
  `max(BACKOFF, GUARD_FRAC·dist)` short per pass so overshoot never forces
  an extra revolution. `cw` means increasing true angle.

#### Config options (`[vortac_grabber]`)

| Option | Default | Meaning |
|---|---|---|
| `angleSensor` | (required) | name of the `[angle ...]` section for the AS5047D |
| `speed` | `50` | default move speed (deg/s) for closed-loop moves |
| `engage_pos` | `130` | true angle for `VORTAC_ENGAGE` / `engage()` |
| `disengage_pos` | `0` | true angle for `VORTAC_DISENGAGE` / `disengage()` |
| `engage_mode` | `shortest` | direction constraint for engage: `shortest`, `cw`, `ccw` |
| `disengage_mode` | `shortest` | direction constraint for disengage: `shortest`, `cw`, `ccw` |
| `lookup_table` | (saved) | JSON LUT `[[true_deg, raw_deg], ...]` written by `VORTAC_CALIBRATE` |
| `zero_pos_offset` | `0.0` | offset written by `VORTAC_SET_ZERO` |

#### Gcode commands

**`VORTAC_CALIBRATE`** — build and store the LUT (run `SAVE_CONFIG` after):

| Parameter | Default | Meaning |
|---|---|---|
| `SAMPLES` | `180` | bins per revolution (LUT size) |
| `SPEED` | `40` | sweep speed (deg/s) |
| `TURNS` | `2` | revolutions **per direction**; last turn wins |
| `DIR` | `both` | `cw`, `ccw`, or `both` (cw + ccw, midpoint stored, backlash reported) |
| `PHASE` | `step/2` | pre-roll so the sensor seam falls between bins |
| `SETTLE` | `0.10` | dwell (s) after each step before reading |
| `READS` | `8` | direct SPI reads per bin (circular-meaned) |
| `READ_DWELL` | `0.001` | pause (s) between the reads of one bin |

**`VORTAC_MOVE`** — closed-loop move to an absolute true angle:

| Parameter | Default | Meaning |
|---|---|---|
| `TARGET` | `0.0` | target true angle (deg) |
| `MODE` | `shortest` | `shortest`, `cw`, or `ccw` |
| `SPEED` | config `speed` | move speed (deg/s) |
| `TOL` | `2.0` | acceptance tolerance (deg, shortest-distance) |
| `BACKOFF` | `0.5·TOL` | cw/ccw only: minimum stop-short distance per pass (clamped < TOL) |
| `GUARD_FRAC` | `0.05` | cw/ccw only: stop short by this fraction of the remaining distance |
| `MAX_ITERS` | `10` | maximum measure/move passes |
| `READS` | `2` | sensor reads per measurement (circular median) |
| `READ_SETTLE` | `0.010` | dwell (s) before each measurement |

**`VORTAC_ENGAGE [ANGLE=<deg>]`** / **`VORTAC_DISENGAGE`** — closed-loop
move to `engage_pos` (or `ANGLE`) / `disengage_pos`, using
`engage_mode` / `disengage_mode` as the direction constraint.

**`VORTAC_SET_ZERO`** — store the current LUT-corrected angle as
`zero_pos_offset` (run `SAVE_CONFIG` after). Approach the reference
position in the operating direction if you use cw/ccw modes.

**`VORTAC_SIMPLE_READ`** — print raw and true angle once.

**`VORTAC_MESURE`** — diagnostic sweep over one revolution, prints
`rawPairs` / `lutPairs` / `finalPairs` for `dev_scripts/plotData.py`:

| Parameter | Default | Meaning |
|---|---|---|
| `SAMPLES` | `90` | measurement points over one revolution |
| `SPEED` | `60` | sweep speed (deg/s) |
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
  tweaks belong in `tool_activate_gcode` / `tool_deactivate_gcode`. Saved dock
  positions are the hooked/engage position; load uses saved Z then lifts to
  `Z + DOCK_Z_CLEARANCE`, and unload approaches at `Z + DOCK_Z_CLEARANCE`
  before dropping to saved Z.
- **Detection:** `VORTAC_DETECT` turns all dock Tool_id channels off, strobes
  one dock's configured LED channel on at a time, and assigns the dock to any
  tool whose cached `dock_sense_pin` reads LOW. Every LED's original RGBW value
  is restored afterward. The grabbed tool is detected from `grab_sense_pin`.
  Detection aborts if any tool already reads dock LOW while all dock Tool_id
  channels are off.
- **Dock calibration context:** `VORTAC_SELECT_DOCK DOCK=dockN` runs detection
  and stores the detected dock/tool pair. `VORTAC_DOCK_CAL_SAVE` saves the
  current hooked/engage XYZ to that selected tool/dock and fails if no tool was
  detected.

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
