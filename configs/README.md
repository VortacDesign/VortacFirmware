# Configs

Klipper configs for the Vortac toolchanger. Files under `vortac_configs/` are
bind-mounted into `~/printer_data/config/vortac_configs/` by `install.sh` and
reload after `git pull` + `systemctl restart klipper`.

**User-specific configs follow the `.example` pattern:** the repo only ships
`*.example.cfg` templates; the real files (`printer.cfg`, `tools.cfg`, every
`tools/*.cfg`) are gitignored. Copy `<name>.example.cfg` to `<name>.cfg` and
edit — Klipper never reads `.example` files, it only loads what `printer.cfg`
pulls in via `[include]`. General hardware configs (everything under `mcu/`)
stay tracked as-is.

## Layout

```
configs/
├── printer.example.cfg          template for your printer.cfg (real one gitignored)
└── vortac_configs/
    ├── mcu/
    │   ├── octopus.cfg          mainboard: XY, Z×4 sensorless, dock LEDs
    │   ├── vortac.cfg           grabber MCU + [vortac_grabber] + [vortac_qgl_state]
    │   ├── SB2209.cfg           toolboard PCB template — reused by every SB2209 tool
    │   └── kraken.cfg           placeholder for future mainboard
    ├── tools/
    │   └── tool0.example.cfg    template: static [mcu tool0] + SB2209 include
    │                            + [vortac_tool T0] (real tools/*.cfg gitignored)
    └── tools.example.cfg        template: per-tool includes + [vortac_manager]
                                 (real tools.cfg gitignored)
```

## Boot order (`printer.cfg`)

```ini
[include vortac_configs/mcu/octopus.cfg]   # mainboard
[include vortac_configs/mcu/vortac.cfg]    # grabber MCU + qgl_state
[include vortac_configs/tools.cfg]         # tools + manager
```

## Adding a tool — the checklist

The trailing number is the convention that links everything: `tool1` ↔ `T1` ↔
`dock1`. `tool_index`, `mcu_name`, `home_dock` and `mcu_from` are all derived
from it — you never write them.

1. **Get the board's CAN UUID** (every board has a unique one):
   `~/klipper/scripts/canbus_query.py can0`
2. **Copy a tool file**: `cp tools/tool0.cfg tools/tool1.cfg`
3. **Replace every `tool0` → `tool1`, `T0` → `T1`, `dock0` → `dock1`.**
   The number appears in exactly three section headers (`[mcu tool1]`,
   `[include_with tool1 …]`, `[vortac_tool T1]`), the sense-pin prefixes
   (`tool1:PB…`), the `params_dock1_*` keys, and
   `ACTIVATE_EXTRUDER EXTRUDER=extruder1`. **Grep the file afterwards** for
   the old number — a missed spot does NOT error, it silently merges into the
   other tool's sections (Klipper merges duplicate sections, last one wins;
   the symptom is one franken-tool, e.g. one board shut down by the other
   tool's sensor config).
4. **Set the new `canbus_uuid`** under `[mcu tool1]`.
5. **Add `[include tools/tool1.cfg]` to `tools.cfg`** and make sure
   `dock_count` covers the new dock.
6. **Restart Klipper**. `vortac_manager` refuses to start (with a clear
   message) on duplicated `tool_index`/`mcu_name`/`canbus_uuid`/sense pins or
   an `mcu_name` without a matching `[mcu …]` section — fix what it names.
7. **Verify**: `VORTAC_STATUS` lists both tools; `VORTAC_DETECT DEBUG=1`
   shows each tool flip only on its own dock's strobe.
8. **Calibrate the dock position** (next section), then `SAVE_CONFIG`.

Why the remaining duplication exists: `[mcu toolN]` must be declared
statically because Klipper registers MCU pin chips before extras like
`include_with` run — it cannot be injected. And pins must name their chip
(`tool1:PB7`), which is plain Klipper syntax.

`include_with` reads the PCB template once per tool, swaps the MCU namespace,
rewrites every `EBBCan:` pin reference, prefixes named sections with the
namespace (`[neopixel logo_rgb]` → `[neopixel tool1_logo_rgb]` — so LEDs are
addressable per tool: `SET_LED LED=tool1_logo_rgb …`), and renames the Klipper
singletons for index ≥ 1 (`[extruder]` → `[extruder1]`, `[fan]` →
`[fan_generic tool1_fan]`).

The namespace is also the display name dashboards show. Prefer pretty names?
Name the MCU after the physical tool — then set the two derived options
explicitly (a namespace without a trailing number can't derive them):

```ini
[mcu miniPink]
canbus_uuid: …

[include_with miniPink vortac_configs/mcu/EBB42_V12.cfg]
tool_index: 0                # required: no trailing number in the namespace
skip_sections:
  mcu EBBCan

[vortac_tool T0]
mcu_name: miniPink           # default would be tool0
dock_sense_pin: miniPink:PB7
grab_sense_pin: miniPink:PB5
```

Dashboards then show `miniPink_logo_rgb`, `miniPink_hotend_fan`, etc. Only
`extruder`/`extruder1` stay index-based (hardwired by Klipper), and the
`T0`/`T1` commands plus `dock0`/`dock1` names are fixed too (slicers emit
`T<n>`; docks map to LED chain indices). Macros can still address per-tool
hardware uniformly via the tool's `mcu_name`, e.g. in
`tool_activate_gcode`: `SET_LED LED={tool.mcu_name}_logo_rgb …`. The template `[mcu EBBCan]` is skipped with
`skip_sections:` because each tool file owns its real `[mcu toolN]`.

### Per-tool hardware deviations (`overrides:`)

Never edit the shared PCB template for one tool. Deviations go on that tool's
`[include_with]` section, one `section.key: value` per line:

```ini
[include_with tool0 vortac_configs/mcu/EBB42_V12.cfg]
skip_sections:
  mcu EBBCan
overrides:
  extruder.sensor_type: MAX31865
  extruder.sensor_pin: EBBCan: PA4
  extruder.spi_bus: spi1
  extruder.rtd_nominal_r: 1000
  extruder.rtd_reference_r: 4300
  extruder.rtd_num_of_wires: 2
  extruder.rtd_use_50Hz_filter: True
```

Section names are the ORIGINAL template names (pre-rename); pin values may be
written template-relative (`EBBCan: PA4` — rewritten automatically). Keys the
template lacks are added; an empty value deletes the option. RTD pairing rule
of thumb: PT100 → `rtd_reference_r: 430`, PT1000 → `4300` (check the reference
resistor on your board; a mismatch shows up as a `Thermocouple reader fault`
MCU shutdown).

## Where the old `tool_doc_load`/`unload` jinja blocks went

- **XYZ approach/depart** is hardcoded in `vortac_manager.py`
  (constants `DOCK_Y_SAFE`, `DOCK_Z_CLEARANCE`, feedrates at the top).
- **Per-tool warm-up / cool-down** → `tool_activate_gcode` /
  `tool_deactivate_gcode` on `[vortac_tool Tn]`.
- **FLAT / TILT** of the gantry around dock approach is automatic.
- **Per-tool offsets** → `gcode_offset_x/y/z` on `[vortac_tool Tn]`; manager
  applies them via `SET_GCODE_OFFSET` on every tool change.

## Homing, QGL & probing — the gantry workflow

The 4-Z gantry has two valid states (tracked by `[vortac_qgl_state]`):
**flat** (top-home/frame reference — dock geometry valid) and **tilted**
(bed reference — print geometry valid). Rules:

- **Z homing** (against the top/frame) resets the state to `flat` and clears
  the stored QGL deltas — after any fresh Z home, QGL must be re-run before
  printing.
- **All probing requires NO tool held.** The probe touch point sits above the
  nozzle tip whenever a tool is grabbed, so the nozzle would hit the bed
  before the probe triggers. `vortac_manager` enforces this at the probe
  object itself: `QUAD_GANTRY_LEVEL`, `BED_MESH_CALIBRATE`, `PROBE`,
  `PROBE_ACCURACY`, … all refuse while a tool is held (logical state or
  `grab_sense`). Park the tool first: `VORTAC_UNLOAD`.
- **Tool changes handle FLAT/TILT automatically**: the manager flattens the
  gantry for the dock motion and re-tilts afterwards — no re-probing needed.

Typical print start (no tool held yet):

```
G28                  # top-home -> flat
QUAD_GANTRY_LEVEL    # no tool! -> tilted
BED_MESH_CALIBRATE   # still no tool
T0                   # manager: FLAT -> fetch -> TILT, applies offsets
# print
```

If Z was re-homed mid-session while a tool is held: `VORTAC_UNLOAD`, then
`QUAD_GANTRY_LEVEL`, then pick the tool back up.

## Calibrating a dock position

```
VORTAC_GANTRY_FLAT
G28                           # if not homed
VORTAC_SELECT_DOCK DOCK=dock0 # detects the tool in dock0 and selects it
# jog toolhead to the exact hooked/engage position in the dock
VORTAC_DOCK_CAL_SAVE
SAVE_CONFIG
```

`VORTAC_SELECT_DOCK` runs `VORTAC_DETECT`, stores the detected dock/tool pair,
and fails if the selected dock has no detected tool. `VORTAC_DOCK_CAL_SAVE`
refuses if the gantry is `tilted` (dock geometry only valid frame-flat) or no
dock/tool was selected. The saved Z is the hooked/engage height. Tool loading
uses that saved Z directly, then lifts by `DOCK_Z_CLEARANCE`; unloading
approaches at saved Z + `DOCK_Z_CLEARANCE`, drops to saved Z, then disengages.

Manual fallback:

```
VORTAC_DOCK_SAVE_POS TOOL=T0 DOCK=dock0
SAVE_CONFIG
```

## Gcode quick reference

| Command | Module | Purpose |
|---|---|---|
| `T0`, `T1`, ... | manager | tool change |
| `VORTAC_STATUS` | manager | report current tool, dock map, QGL state |
| `VORTAC_DETECT` | manager | strobe dock Tool_id channels and update dock map |
| `VORTAC_SELECT_DOCK DOCK=dockN` | manager | detect/select dock and its tool for calibration |
| `VORTAC_DOCK_CAL_STATUS` | manager | report selected calibration dock/tool |
| `VORTAC_DOCK_CAL_SAVE` | manager | save current hooked/engage XYZ for selected calibration dock/tool |
| `VORTAC_LOAD TOOL=Tn` | manager | fetch Tn from its dock |
| `VORTAC_UNLOAD` | manager | park the held tool at its dock |
| `VORTAC_SET_CURRENT_TOOL TOOL=Tn` | manager | set logical current tool without movement |
| `VORTAC_SET_CURRENT_TOOL CLEAR=1` | manager | clear logical current tool |
| `VORTAC_DOCK_SAVE_POS TOOL=Tn DOCK=name` | manager | save current hooked/engage XYZ as tool's pos for `name` |
| `VORTAC_GANTRY_FLAT` / `VORTAC_GANTRY_TILT` | qgl_state | toggle gantry between frame- and bed-flat |
| `VORTAC_QGL_STATUS` | qgl_state | report gantry state and stored QGL deltas |
| `VORTAC_SENSE_STATUS` | manager | raw cached dock/grab sense pin states per tool |
| `VORTAC_SENSE_MONITOR [DURATION=s]` | manager | poll sense states live, report transitions |
| `VORTAC_DOCK_STROBE DOCK=dockN VALUE=0..1` | manager | manually set one dock strobe channel (debug) |
| `VORTAC_CALIBRATE [DIR=cw\|ccw\|both]` | grabber | populate AS5047D LUT (default: both directions, reports backlash) |
| `VORTAC_SET_ZERO` | grabber | set zero offset to current angle |
| `VORTAC_MOVE TARGET=deg [MODE=shortest\|cw\|ccw]` | grabber | closed-loop angle move |
| `VORTAC_ENGAGE [ANGLE=deg]` | grabber | closed-loop move to `engage_pos` |
| `VORTAC_DISENGAGE` | grabber | closed-loop move to `disengage_pos` |
| `VORTAC_SIMPLE_READ` | grabber | read raw + true angle |
| `VORTAC_MESURE [SAMPLES=n]` | grabber | LUT sweep dump (debug) |

For module internals, see [`klippy/extras/README.md`](../klippy/extras/README.md).
