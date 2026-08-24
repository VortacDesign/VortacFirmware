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

- **`tool_index`** is assigned by LOAD ORDER for tool templates: when the
  template carries a (non-skipped) `[vortac_tool …]` section, each
  `[include_with]` takes the next value from a per-printer counter — the
  first tool include in `tools.cfg` is 0, the next 1, … Commenting a tool
  out renumbers the ones after it, which keeps Klipper's
  `extruder`/`extruder<N>` contract intact. Explicit `tool_index:` wins;
  templates without a `vortac_tool` section fall back to the namespace's
  trailing number (`tool1` → 1, else the option is required). Mixing
  explicit indices with auto-numbered includes is unsupported (the counter
  ignores explicit values).
- **`[vortac_tool TN]` injection:** the template's `vortac_tool` placeholder
  is renamed to `[vortac_tool <namespace>]` (PURE name, no index — this is
  the persistence anchor), and `tool_index`, `mcu_name` (= namespace),
  `display_name` (`<namespace><index>`, the dashboard prefix of the
  injected hardware sections), `canbus_uuid` (from the static
  `[mcu <namespace>]` section) and `extruder_name` (the renamed extruder)
  are auto-injected. Template values and overrides win over the injection.
  `home_dock` is optional (docks come from `VORTAC_DETECT` at runtime); it
  can be pinned via `overrides:` (`vortac_tool TN.home_dock: dockX`).
  If SAVE_CONFIG's autosave block already created the section
  (persisted `params_*` dock positions), the injection MERGES into it —
  calibration data is preserved.
- **Section rename rules.** EVERY named section gets the DISPLAY prefix
  `<namespace><tool_index>` — that is what Mainsail/Fluidd/KlipperScreen
  show, and the index makes `extruder1` ↔ `miniGrey1_*` visually matchable
  (`miniPink` + index 0 → `miniPink0_logo_rgb`; legacy digit-suffixed
  namespaces like `tool1` stay as-is). Exceptions: `[mcu]` and
  `[vortac_tool]` keep the pure namespace. The display prefix shifts with
  order-based renumbering — safe, because the renamed hardware sections
  never hold SAVE_CONFIG data; address them index-proof in macros via
  `{tool.display_name}_…`. Reference sections
  (`tmcXXXX`, `verify_heater`) follow their target's rename. The Klipper
  singletons `[extruder]` and `[fan]` are only renamed for `tool_index ≥ 1`
  (tool 0 keeps the primary extruder and the M106 fan), and multi-extruder
  naming is hardwired to `extruder<N>` by Klipper regardless of namespace —
  `[extruder miniPink]` is not a valid Klipper section:
  | Original | Tool 0 (ns `tool0`) | Tool 1 (ns `miniGrey`) |
  |---|---|---|
  | `[mcu <from>]` | `[mcu tool0]` | `[mcu miniGrey]` |
  | `[vortac_tool <any>]` | `[vortac_tool tool0]` | `[vortac_tool miniGrey]` |
  | `[<head> name]` (generic) | `[<head> tool0_name]` | `[<head> miniGrey1_name]` |
  | `[adxl345 (name)]` | `[adxl345 tool0(_name)]` | `[adxl345 miniGrey1(_name)]` |
  | `[extruder]` | `[extruder]` | `[extruder1]` |
  | `[tmcXXXX extruder]` | `[tmcXXXX extruder]` | `[tmcXXXX extruder1]` |
  | `[verify_heater extruder]` | `[verify_heater extruder]` | `[verify_heater extruder1]` |
  | `[fan]` | `[fan]` | `[fan_generic miniGrey1_fan]` |

  The generic rule covers `neopixel`, `heater_fan`, `temperature_sensor`,
  `output_pin`, `filament_*_sensor`, `gcode_macro`, `manual_stepper`, … —
  anything with a name part. Bare singletons without a name part
  (`input_shaper`, `firmware_retraction`, …) cannot be namespaced and pass
  through unchanged; `skip_sections` them if they collide across tools.
- **Value rewrite:** any `<from>:` pin token in option values becomes `<to>:`;
  whole-string section refs (e.g. `heater: extruder` → `heater: extruder1`)
  are also fixed up.
- **Skip-list** for `tool_index ≥ 1`: `resonance_tester`, `input_shaper`,
  `shaketune` (Klipper singletons that can't be renamed). Config sections can
  also pass `skip_sections:` as a comma/newline list; tool configs use this to
  skip the template `[mcu EBBCan]` because `[mcu toolN]` must be declared
  statically.
- **Overrides** (per-tool deviations from the shared template) via the
  `overrides:` config option, one `section.key: value` per line:
  ```ini
  overrides:
    extruder.sensor_type: MAX31865
    extruder.sensor_pin: EBBCan: PA4
  ```
  Section names are the ORIGINAL template names; values run through the
  MCU/value rewriter (template-relative pins allowed). Keys the template
  lacks are added; an empty value deletes the option. A JSON dict
  (`{"section": {"key": "value"}}`, null deletes) is also accepted, and is
  the format of the programmatic `overrides=` arg.
- **Programmatic API** (used by `vortac_tool`):
  `include_with_remap(printer, parent_config, filepath, namespace,
  mcu_from=None, tool_index=0, overrides=None, skip_sections=None,
  template=None)`.

### `vortac_tool.py` — `[vortac_tool <name>]`

Per-tool logical definition. Normally injected by `include_with` from the
PCB template's `[vortac_tool TN]` placeholder (identity auto-filled, see
above); a statically written `[vortac_tool T0]` still works. The tool
hardware must already be present in normal Klipper config (`[mcu <name>]`
plus `[include_with <name> ...]`) before this extra loads; Klipper registers
MCU pin chips before extras run.

- **Identity:** injected sections get `tool_index`, `mcu_name` and
  `extruder_name` from `include_with`; static `[vortac_tool T1]` sections
  derive them from the trailing number (`T1` → index 1, `tool1`,
  `extruder1`, `home_dock=dock1`). `home_dock` is NOT derived from the
  (order-based, unstable) tool_index and is optional — docks are
  established at runtime by `VORTAC_DETECT`; a manual fallback can be
  pinned via include_with overrides.
- **Availability & ghosts:** `available:` defaults to `True` only when BOTH
  sense pins are configured. A leftover autosave-only section (a
  commented-out tool whose SAVE_CONFIG'd `params_*` dock positions remain in
  `printer.cfg`) therefore loads harmlessly as a "ghost": `tool_index=None`,
  `available=False`, calibration preserved until the tool file returns. A
  genuinely sense-pin-less tool (degraded blind-unhook mode) must set
  `available: True` explicitly.
- **Template loading:** lives in the tool config via `[include_with ...]`, not
  in `vortac_tool.py`.
- **Per-dock storage:** `params_<dock>_<x|y|z>` in this section (persisted
  via `configfile.set` + SAVE_CONFIG; keyed by tool NAME, so calibration
  survives renumbering). Manager API: `get_dock_pos(dock, axis)`,
  `has_dock_pos(dock)`, `save_dock_pos(dock, x, y, z)`.
- **Templates:** `tool_activate_gcode`, `tool_deactivate_gcode` — run by
  manager around the dock motion. Jinja context: `tool` (this VortacTool),
  `dock=None`, `params`.

### `vortac_manager.py` — `[vortac_manager]`

Coordinator. At `klippy:connect` it discovers `[vortac_tool *]` (only those
with `available=True`; ghosts are skipped), the grabber (required), and
`[vortac_qgl_state]` (optional). Registers a `T<tool_index>` command for
every available tool — the command comes from the INDEX, tool ids are
display names like `miniGrey`. `TOOL=` parameters accept the tool name
(case-insensitive), `T<n>`, or a bare index.

- **Identity validation:** startup fails with a clear message if two tools
  share a `tool_index`, `mcu_name`, `canbus_uuid`, or sense pin, if the
  non-ghost tools' indices are not contiguous 0..N-1 (Klipper's
  extruder-naming contract; holds by construction with order-based
  numbering), if an available tool lacks `tool_index`, or if a tool's
  `mcu_name` has no matching `[mcu …]` section. Klipper itself
  merges duplicate config sections silently (last value wins), so a
  copied-but-not-fully-renamed tool file never errors on its own — this
  guard catches the detectable leftovers.

- **Gcode:** `T0/T1/…` (registered dynamically), `VORTAC_STATUS`,
  `VORTAC_LOAD TOOL=<name>|Tn`, `VORTAC_UNLOAD`, `VORTAC_SET_CURRENT_TOOL`,
  `VORTAC_DETECT`, `VORTAC_SELECT_DOCK DOCK=dockN`,
  `VORTAC_DOCK_CAL_STATUS`, `VORTAC_DOCK_CAL_SAVE`,
  `VORTAC_DOCK_SAVE_POS TOOL=<name>|Tn DOCK=dockN`,
  `VORTAC_DOCK_POWER [DOCK=dockN VALUE=0|1 [ONLY=1] [FORCE=1]]`,
  `VORTAC_STATUS_LED [MODE=sense|power|detected]`.
- **Tool change sequence** in `_change_to`:
  `tool_deactivate_gcode` → `VORTAC_GANTRY_FLAT` → `_park_at_dock(current)`
  → `_fetch_from_dock(target)` → `VORTAC_GANTRY_TILT` →
  `ACTIVATE_EXTRUDER EXTRUDER=<extruder_name>` → `SET_GCODE_OFFSET`
  → `tool_activate_gcode`.
- **Dock power (`dock_power_mode`: off | occupancy | handover):** the dock
  LED's red channel gates the parked tool board's supply, INVERTED (red HIGH
  cuts it, 0.0 = powered). The supply is switched around the *Y* moves at the
  dock, never the Z moves — the spring-loaded dock board rides along through
  the Z travel, so Y is where the pogos actually open and close:
  park slides in, verifies `dock_sense` with the dock still dead, then
  energizes before the drop; fetch engages, lifts, verifies `grab_sense`,
  then cuts before the backout. The board is always fed from the other side
  while the dock switches.
  Everything else follows from one constraint: the sense pins live on the
  TOOL's mcu and Klipper has no optional mcus, so a dark board is a klippy
  shutdown. Hence only a dock POSITIVELY known to be empty is de-energized
  (`_policy_power`) — a tool with no sense readings at all, or an
  unavailable tool whose `[mcu ...]` is still included, can never be
  located and therefore pins EVERY dock on — every failure path
  re-energizes, and
  `RESTART`/`FIRMWARE_RESTART` are wrapped to power all docks first
  (`_install_restart_guard` — WS2812 latch across a restart, and
  `mcu_identify` runs before the LED is ever programmed). A full power cycle
  clears the latch and is the unconditional recovery.
  `_power_off_before_backout` probes whether `dock_sense` survives the cut;
  if it does not, the release check would read a false "released", so the
  supply is restored and the dead-break is disabled for the session
  (`sense_needs_dock_power`).
- **Status LED (`argb_status_mode`):** `sense` (mirrors the sense pull),
  `power` (lit while the dock feeds its board), `detected` (full = tool
  detected, `argb_status_dim` = confirmed empty, off = map not trustworthy).
  A detection pass always mirrors the strobe regardless of mode.
- **Probe guard:** at `klippy:connect` the manager wraps the `[probe]`
  object's entry points (`start_probe_session` / `run_probe`), so EVERY
  probe-based operation — `QUAD_GANTRY_LEVEL`, `BED_MESH_CALIBRATE`,
  `PROBE`, `PROBE_ACCURACY`, `PROBE_CALIBRATE`, … — refuses while a tool
  is held (logical `current_tool` OR any tool's `grab_sense` reading
  grabbed). Reason: the probe touch point sits above the nozzle tip when
  a tool is grabbed, so the nozzle would hit the bed before the probe
  triggers. Park the tool (`VORTAC_UNLOAD`) before probing.
- **Dock motion** is hardcoded (constants at top of file):
  `DOCK_Y_SAFE = 50.0`, `DOCK_Z_CLEARANCE = 4.5`, `DOCK_APPROACH_F = 2000`,
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
  channels are off. Detection also ESTABLISHES the dock map used by tool
  changes: a grabbed tool is assigned the first free dock with calibrated
  positions for it (a previous choice — fetch origin or manual `home_dock` —
  is kept while still free and calibrated; a warning is printed when no
  free calibrated dock exists). Park-dock resolution on tool change:
  detection map → fetch origin → optional `home_dock`; before the first
  detection, a tool change without `home_dock` errors with
  "run VORTAC_DETECT".
- **Dock calibration context:** `VORTAC_SELECT_DOCK DOCK=dockN` runs detection
  and stores the detected dock/tool pair. `VORTAC_DOCK_CAL_SAVE` saves the
  current hooked/engage XYZ to that selected tool/dock and fails if no tool was
  detected.

## Lifecycle & dependencies

```
config parse:
  [vortac_grabber]    → register grabber gcode commands
  [vortac_qgl_state]  → register FLAT/TILT (hooks attach at klippy:ready)
  [mcu <name>]        → register tool MCU pin chip early
  [include_with ...]  → take next order-based tool_index, inject remapped
                        toolboard sections (incl. [vortac_tool <name>])
  [vortac_manager]    → register VORTAC_LOAD/UNLOAD/STATUS/DOCK_SAVE_POS;
                        defer rest to klippy:connect

klippy:ready    → vortac_qgl_state hooks QGL.adjust_steppers + QUAD_GANTRY_LEVEL
klippy:connect  → vortac_manager finds tools/grabber/qgl, validates identities,
                  registers T<index> commands
```

Hard dependencies:
- `vortac_manager` requires `[vortac_grabber]`.
- Each active tool requires a static `[mcu <name>]` section before `[include_with]`.
- `vortac_qgl_state` requires `[quad_gantry_level]` to be defined.

For config layout and the user-side workflow, see
[`configs/README.md`](../../configs/README.md).
