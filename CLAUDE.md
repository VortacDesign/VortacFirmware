# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

Klipper plugins and config files for the **Vortac Toolchanger** (custom 3D-printer toolchanger with CAN-bus tool boards, angle-sensor-driven grabber, and ARGB-based dock detection). This is **not** a standalone application — the Python files are loaded into Klipper at runtime, and the configs are consumed by Klipper at printer startup.

The primary branch is `development` (see `install.sh`'s default), not `main`.

## Deployment model (no build, no tests)

There is **no build step, no test suite, and no linter configured**. Code runs inside Klipper on a Raspberry Pi.

`install.sh` (run once on the Pi as root) sets up:
1. **Symlinks** every `*.py` in `klippy/extras/` into `$KLIPPER_DIR/klippy/extras/` (top-level only; no recursion).
2. A **systemd bind-mount + automount** that exposes `configs/vortac_configs/` at `$HOME/printer_data/config/vortac_configs/`.
3. A **systemd `.path` unit** watching `.git/refs/heads/$BRANCH` and `.git/packed-refs` — every `git pull` re-runs `install.sh` automatically.

So: editing a `.py` here, pushing, and pulling on the Pi makes Klipper pick up the change after `sudo systemctl restart klipper`. There is nothing to "compile" or "package."

`uninstall.sh` reverses everything (removes systemd units, unmounts, deletes symlinks).

`dev_scripts/push_custom_scripts.py` is a quick `sshpass`+`scp` pusher targeting a hardcoded Pi (`192.168.0.188`, user `pi`) — used during ad-hoc iteration when you don't want to commit/pull.

## Code architecture

### Klipper plugin contract (important)

Every file in `klippy/extras/` is a Klipper extra. Klipper imports it when it sees a config section matching the file name. The factory functions Klipper looks for are:

- `load_config(config)` — for singleton sections like `[vortac_grabber]`.
- `load_config_prefix(config)` — for prefix sections like `[include_with foo bar]` or `[vortac_tool T0]`.

Inside an extra, the standard pattern is: store `config.get_printer()`, look up shared objects via `printer.lookup_object('gcode' | 'toolhead' | 'configfile' | 'force_move' | ...)`, register gcode handlers via `gcode.register_command(...)`, and persist tuned values back via `configfile.set(section, key, value)` (user runs `SAVE_CONFIG` to write them).

### `klippy/extras/vortac_grabber.py`

Monolithic grabber controller. Currently mixes three concerns: (1) AS5047D angle-sensor SPI + LUT-based raw→true angle interpolation, (2) closed-loop angle moves on a `manual_stepper` named `grabber`, and (3) tool dock load/unload gcode templates.

Notable internals:
- **`read_raw`** builds 16-bit AS5047D commands (with parity) and shifts them through the angle sensor's SPI helper. Returns the angle in degrees, 14-bit precision.
- **`_measured_to_true`** does wrap-aware circular interpolation across the LUT stored in `lookup_table` (a JSON list of `[true_deg, raw_deg]` pairs). Handles seam crossings.
- **`cmd_vortac_calibrate`** drives the stepper through `TURNS` revolutions, subscribes to the angle sensor's bulk stream via `add_client(cb)`, and fits a per-bin circular mean over the dwell window. **Bulk angle data from Klipper is in radians×10000** (note in `dev_scripts/README.md`) — the conversion `(raw / 10000.0) * RAD2DEG` matters and is easy to miss.
- **`cmd_simple_move`** is a closed-loop seek: bold coarse hop with a safety buffer, then proportional/halving fine convergence using `force_move.manual_move` + repeated quiet reads. Direction-constrained by `MODE=shortest|cw|ccw`.

Persisted state lives in the config section itself: `lookup_table`, `zero_pos_offset`, and per-tool `params_dockN_x/y/z` hooked/engage positions are written via `configfile.set` and survive `SAVE_CONFIG`.

### `klippy/extras/include_with.py`

Implements `[include_with <namespace> <filename>]`. It reads `<filename>` relative to the active Klipper config directory, iterates every section/option, and re-injects them into the calling config with MCU name remapping, value rewrites, section renaming, optional overrides, and optional `skip_sections`.

This is the mechanism by which one shared template can be loaded multiple times under different MCU names. Tool files must still declare `[mcu toolN]` statically before `[include_with ...]`, because Klipper registers MCU pin chips before extras run.

### Configs (`configs/vortac_configs/`)

Live-mounted into Klipper at runtime. Layout:
- `mcu/octopus.cfg` — mainboard (steppers, bed, fans, neopixel docks).
- `mcu/vortac.cfg` — grabber MCU (CAN UUID `70d72bfb79f1`), `[manual_stepper grabber]` + AS5047D wiring + `[vortac_grabber]` hardware settings and `[vortac_qgl_state]`.
- `mcu/SB2209.cfg` — toolhead-board template (extruder, hotend fan, ADXL345, hotend ARGB). Generic MCU name `EBBCan`; intended to be loaded via `include_with` with per-tool namespacing.

`configs/klipper_screen_configs/` — KlipperScreen UI overrides.

## Active refactor (read before making structural changes)

`.claude/refractorPlan.md` is the source of truth for the in-progress redesign. Headline plan:

- **Split** `vortac_grabber.py` into `vortac_grabber.py` (hardware only — angle sensor, stepper, calibration, plus a Python API: `engage()`, `disengage()`, `read_angle()`) and two new modules: `vortac_tool.py` (logical per-tool `[vortac_tool Tn]` sections) and `vortac_manager.py` (coordinator: `T0/T1/...`, dock state machine, `VORTAC_DETECT`).
- **Enhance** `include_with.py` with MCU-name remapping in section headers *and* option values (not just `pin` keys), tool-index-based section auto-rename to dodge Klipper singleton collisions (`[extruder]` → `[extruder1]`, `[fan]` → `[fan_generic toolN_fan]`, etc.), config-value overrides, dynamic config-dir paths, and configurable section skipping.
- **Tool detection** uses a **strobe-by-subtraction** scheme: every tool has two GPIO sense pins (`dock_sense_pin`, `grab_sense_pin`). Normal parked operation keeps dock Tool_id channels ON. Detection temporarily turns all dock Tool_id channels OFF, turns one dock ON at a time (~100 ms), polls all tools' `dock_sense`, and the one that flips identifies the dock. The grabber pulls `grab_sense` LOW on the held tool.
- **Section-renaming table** for the include_with rewriter is in the plan doc — consult it before adding new section types.

The refactor is in progress on the development branch; check current git status before assuming which config files are staged or deployed.

## Conventions worth knowing

- The codebase is bilingual in comments — German and English freely mixed (`Zugriff auf ToolHead`, `Nun die Move-Command`). Don't "translate" existing comments unless asked.
- Persisted tunables go through `configfile.set(self.name, key, value)` and require the user to run `SAVE_CONFIG`. Don't write to disk directly.
- `force_move.manual_move(stepper, distance, speed)` followed by `toolhead.wait_moves()` (and a small `dwell` to let the sensor settle) is the standard pattern for nudging the grabber stepper. Don't try to drive it through the kinematics path.
- Angle math is on a circle — always wrap to `[0, 360)` and handle seam crossings. The existing `wrap360` / `wrap180` / `circ_mean` helpers in `vortac_grabber.py` are the reference implementations.
