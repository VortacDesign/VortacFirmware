# Configs

Klipper configs for the Vortac toolchanger. Files under `vortac_configs/` are
bind-mounted into `~/printer_data/config/vortac_configs/` by `install.sh` and
reload after `git pull` + `systemctl restart klipper`. `configs/printer.cfg`
is gitignored — it's a local reference; the real one lives on the Pi.

## Layout

```
configs/
├── printer.cfg                  (gitignored, local reference)
└── vortac_configs/
    ├── mcu/
    │   ├── octopus.cfg          mainboard: XY, Z×4 sensorless, dock LEDs
    │   ├── vortac.cfg           grabber MCU + [vortac_grabber] + [vortac_qgl_state]
    │   ├── SB2209.cfg           toolboard PCB template — reused by every SB2209 tool
    │   └── kraken.cfg           placeholder for future mainboard
    ├── tools/
    │   ├── tool0.cfg            static [mcu tool0] + SB2209 include + [vortac_tool T0]
    │   └── miniStealth.cfg      placeholder
    └── tools.cfg                top-level: includes per-tool files + [vortac_manager]
```

## Boot order (`printer.cfg`)

```ini
[include vortac_configs/mcu/octopus.cfg]   # mainboard
[include vortac_configs/mcu/vortac.cfg]    # grabber MCU + qgl_state
[include vortac_configs/tools.cfg]         # tools + manager
```

## Adding another SB2209-based tool

1. `cp tools/tool0.cfg tools/tool1.cfg`
2. Replace every `T0`/`tool0`/`dock0` with `T1`/`tool1`/`dock1`
3. Bump `tool_index: 0` → `1`, set the new `[mcu tool1] canbus_uuid`
4. Set placeholder `params_dock1_x/y/z` (calibrate later)
5. Add `[include tools/tool1.cfg]` to `tools.cfg`

Each tool config declares its `[mcu toolN]` statically first. That is required
because Klipper registers MCU pin chips before extras like `include_with` or
`vortac_tool` run.

`include_with` then reads `vortac_configs/mcu/SB2209.cfg` once per tool, swaps
the MCU namespace, rewrites every `EBBCan:` pin reference, and renames sections
per `tool_index` (`[extruder]` → `[extruder1]`, `[fan]` →
`[fan_generic tool1_fan]`, etc.). The template `[mcu EBBCan]` is skipped with
`skip_sections:` because each tool file owns its real `[mcu toolN]`.

## Where the old `tool_doc_load`/`unload` jinja blocks went

- **XYZ approach/depart** is hardcoded in `vortac_manager.py`
  (constants `DOCK_Y_SAFE`, `DOCK_Z_CLEARANCE`, feedrates at the top).
- **Per-tool warm-up / cool-down** → `tool_activate_gcode` /
  `tool_deactivate_gcode` on `[vortac_tool Tn]`.
- **FLAT / TILT** of the gantry around dock approach is automatic.
- **Per-tool offsets** → `gcode_offset_x/y/z` on `[vortac_tool Tn]`; manager
  applies them via `SET_GCODE_OFFSET` on every tool change.

## Calibrating a dock position

```
VORTAC_GANTRY_FLAT
G28                           # if not homed
VORTAC_SELECT_DOCK DOCK=dock0 # detects the tool in dock0 and selects it
# jog toolhead to the exact dock seat
VORTAC_DOCK_CAL_SAVE
SAVE_CONFIG
```

`VORTAC_SELECT_DOCK` runs `VORTAC_DETECT`, stores the detected dock/tool pair,
and fails if the selected dock has no detected tool. `VORTAC_DOCK_CAL_SAVE`
refuses if the gantry is `tilted` (dock geometry only valid frame-flat) or no
dock/tool was selected.

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
| `VORTAC_DOCK_CAL_SAVE` | manager | save current XYZ for selected calibration dock/tool |
| `VORTAC_LOAD TOOL=Tn` | manager | fetch Tn from its dock |
| `VORTAC_UNLOAD` | manager | park the held tool at its dock |
| `VORTAC_SET_CURRENT_TOOL TOOL=Tn` | manager | set logical current tool without movement |
| `VORTAC_SET_CURRENT_TOOL CLEAR=1` | manager | clear logical current tool |
| `VORTAC_DOCK_SAVE_POS TOOL=Tn DOCK=name` | manager | save current XYZ as tool's pos for `name` |
| `VORTAC_GANTRY_FLAT/TILT/STATUS` | qgl_state | toggle gantry between frame- and bed-flat |
| `VORTAC_CALIBRATE` | grabber | populate AS5047D LUT |
| `VORTAC_SET_ZERO` | grabber | set zero offset to current angle |
| `VORTAC_MOVE TARGET=deg` | grabber | closed-loop angle move |
| `VORTAC_SIMPLE_READ` | grabber | read raw + true angle |
| `VORTAC_MESURE` | grabber | LUT sweep dump (debug) |

For module internals, see [`klippy/extras/README.md`](../klippy/extras/README.md).
