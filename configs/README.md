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
    │   ├── EBB42_V12.cfg        toolboard PCB template — reused by every EBB42 tool
    │   └── kraken.cfg           placeholder for future mainboard
    ├── tools/
    │   └── tool0.example.cfg    template: static [mcu <name>] + PCB-template
    │                            include (real tools/*.cfg gitignored)
    └── tools.example.cfg        template: per-tool includes + [vortac_manager]
                                 (real tools.cfg gitignored)
```

## Boot order (`printer.cfg`)

```ini
[include vortac_configs/mcu/octopus.cfg]   # mainboard
[include vortac_configs/mcu/vortac.cfg]    # grabber MCU + qgl_state
[include vortac_configs/tools.cfg]         # tools + manager
```

## Tool naming & numbering — the conventions

**The tool's NAME is defined exactly once**: the `[mcu <name>]` section /
`include_with` namespace (e.g. `miniGrey`). Dashboards show the DISPLAY
prefix `<name><index>` (`miniGrey1_logo_rgb`, `miniGrey1_hotend_fan`, …) so
`extruder1` is visually matchable to `miniGrey1_*`. The pure name stays the
persistence key: `[mcu miniGrey]` and `[vortac_tool miniGrey]` never carry
the index, so saved dock calibration survives renumbering.

**The tool's NUMBER comes from the include ORDER in `tools.cfg`**: the first
(uncommented) tool include is `T0`, the next `T1`, and so on. The first tool
always owns Klipper's primary `[extruder]` and `[fan]` (hardwired by Klipper
for `M104`/`M106`), so commenting a broken tool out simply renumbers the rest
— Klipper never sees `[extruder1]` without `[extruder]`.

**The tool's DOCK is established at runtime by `VORTAC_DETECT`**: docked
tools are located via the strobe map, and a grabbed tool is assigned the
first free dock that has calibrated positions for it (a warning is printed
if none exists). Run `VORTAC_DETECT` before the first tool change; a manual
fallback can optionally be pinned per tool via the `overrides:` block
(`vortac_tool TN.home_dock: dockX`). Saved dock positions are keyed by tool
name, so they survive renumbering — and a commented-out tool's calibration
survives in
`printer.cfg`'s autosave block as a harmless ghost `[vortac_tool <name>]`
section (loaded as `available: False`) until the tool returns.

The `[vortac_tool]` section itself is injected from the PCB template's
`[vortac_tool TN]` placeholder; `tool_index`, `mcu_name`, `canbus_uuid` (from
the static `[mcu]` section) and `extruder_name` (the renamed extruder) are
auto-filled. The manager runs `ACTIVATE_EXTRUDER` with `extruder_name` on
every tool change.

## Adding a tool — the checklist

1. **Get the board's CAN UUID** (every board has a unique one):
   `~/klipper/scripts/canbus_query.py can0`
2. **Copy a tool file**: `cp tools/miniStealth_grey.cfg tools/myTool.cfg`
   (or start from `tools/tool0.example.cfg`).
3. **Rename the namespace**: `[mcu myTool]` + `[include_with myTool …]` —
   those two headers are the only places the name appears.
4. **Set the new `canbus_uuid`** under `[mcu myTool]`.
5. **Add `[include tools/myTool.cfg]` to `tools.cfg`** — its position in the
   include list is its tool number. Make sure `dock_count` covers the dock.
6. **Restart Klipper**. `vortac_manager` refuses to start (with a clear
   message) on duplicated `mcu_name`/`canbus_uuid`/sense pins,
   non-contiguous tool indices, or an `mcu_name` without a matching
   `[mcu …]` section — fix what it names.
7. **Verify**: `VORTAC_STATUS` lists both tools; `VORTAC_DETECT DEBUG=1`
   shows each tool flip only on its own dock's strobe (detection also
   establishes the dock map used by tool changes).
8. **Calibrate the dock position** (next section), then `SAVE_CONFIG`.

Why the `[mcu <name>]` section stays in the tool file: Klipper registers MCU
pin chips before extras like `include_with` run — it cannot be injected.

`include_with` reads the PCB template once per tool, swaps the MCU namespace,
rewrites every `EBBCan:` pin reference, prefixes named sections with the
display prefix (`[neopixel logo_rgb]` → `[neopixel miniGrey1_logo_rgb]` — so
LEDs are addressable per tool: `SET_LED LED=miniGrey1_logo_rgb …`, or
index-proof in macros via `{tool.display_name}_logo_rgb`), renames the
Klipper singletons for index ≥ 1 (`[extruder]` → `[extruder1]`, `[fan]` →
`[fan_generic miniGrey1_fan]`), and turns `[vortac_tool TN]` into
`[vortac_tool miniGrey]` (pure name — the persistence anchor).

Only `extruder`/`extruder1` stay index-based (hardwired by Klipper), and the
`T0`/`T1` commands plus `dock0`/`dock1` names are index-based too (slicers
emit `T<n>`; docks map to LED chain indices). Manager commands accept either
form: `VORTAC_LOAD TOOL=miniGrey` and `VORTAC_LOAD TOOL=T1` both work. Macros
can address per-tool hardware uniformly via the tool's `display_name`, e.g.
in `tool_activate_gcode`: `SET_LED LED={tool.display_name}_logo_rgb …`. The
template
`[mcu EBBCan]` is skipped with `skip_sections:` because each tool file owns
its real `[mcu <name>]`.

Legacy/static alternative: a hand-written `[vortac_tool T0]` section still
works when the template carries no `[vortac_tool TN]` placeholder — its
trailing number derives `tool_index=0`, `mcu_name=tool0`, `home_dock=dock0`,
and a trailing-numbered namespace (`tool0`) derives the include's tool_index.

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
  `tool_deactivate_gcode` on `[vortac_tool <name>]` (set via include_with
  overrides: `vortac_tool TN.tool_activate_gcode: …`).
- **Extruder switching** is automatic: the manager runs `ACTIVATE_EXTRUDER`
  with the tool's injected `extruder_name` on every tool change.
- **FLAT / TILT** of the gantry around dock approach is automatic.
- **Per-tool offsets** → `gcode_offset_x/y/z` on `[vortac_tool <name>]`;
  manager applies them via `SET_GCODE_OFFSET` on every tool change.

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
VORTAC_DETECT        # establish the dock map (required before first change)
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
VORTAC_DOCK_SAVE_POS TOOL=miniGrey DOCK=dock0
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
| `VORTAC_LOAD TOOL=<name>\|Tn` | manager | fetch a tool from its dock |
| `VORTAC_UNLOAD` | manager | park the held tool at its dock |
| `VORTAC_SET_CURRENT_TOOL TOOL=<name>\|Tn` | manager | set logical current tool without movement |
| `VORTAC_SET_CURRENT_TOOL CLEAR=1` | manager | clear logical current tool |
| `VORTAC_DOCK_SAVE_POS TOOL=<name>\|Tn DOCK=dockN` | manager | save current hooked/engage XYZ as tool's pos for `name` |
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
