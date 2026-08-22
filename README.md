# Vortac Firmware

Klipper plugins and config files for the **Vortac Toolchanger** — a custom
3D-printer toolchanger with CAN-bus tool boards, an angle-sensor-driven
grabber, and ARGB-based dock detection.

- Module documentation (Klipper extras): [klippy/extras/README.md](klippy/extras/README.md)
- Config layout and boot order: [configs/README.md](configs/README.md)

---

# Setup Guide

Step-by-step guide for bringing up a Vortac toolchanger from scratch — from
flashing a toolhead board to a calibrated grabber.

## 1. Flash the toolhead board

Flash Katapult (bootloader) and Klipper onto the CAN toolhead board.
Follow the excellent guide at:

> https://canbus.esoterical.online/toolhead_flashing.html

After flashing, note the board's **CAN bus UUID** (`~/klippy-env/bin/python
~/klipper/scripts/canbus_query.py can0`) — you will need it in step 3.2.

## 2. Install VortacFirmware on the Pi

If not already installed, either clone the repository directly:

```bash
git clone https://github.com/VortacDesign/VortacFirmware.git ~/vortac
```

or let Moonraker manage it by adding this to `moonraker.conf`
(adjust path/branch if needed):

```ini
[update_manager vortac]
type: git_repo
path: ~/vortac
origin: https://github.com/VortacDesign/VortacFirmware.git
primary_branch: development
managed_services: klipper
```

Then run the installer once:

```bash
cd ~/vortac && sudo ./install.sh
```

The installer symlinks the Klipper plugins into `klippy/extras/`, mounts the
config files into `~/printer_data/config/vortac_configs/`, and sets up a
systemd path trigger that re-runs itself on every `git pull` — so this is
normally a one-time step.

> **Note:** If `vortac_configs/` shows up empty in your config folder, run
> `install.sh` manually once more, then clear your browser cache and reload
> the web interface (Mainsail/Fluidd caches the file tree).

## 3. Adapt the configs to your setup

Include the Vortac configs from your `printer.cfg` in this order:

```ini
[include vortac_configs/mcu/octopus.cfg]   # mainboard
[include vortac_configs/mcu/vortac.cfg]    # grabber MCU + [vortac_grabber] + [vortac_qgl_state]
[include vortac_configs/tools.cfg]         # tools + [vortac_manager]
```

### 3.1 Pick the right mainboard

Choose the include that matches your mainboard from
`vortac_configs/mcu/` (currently `octopus.cfg`; `kraken.cfg` is a
placeholder). Verify stepper/fan/LED pins against your wiring.

### 3.2 Set up `tools.cfg`

`vortac_configs/tools.cfg` is the tool registry: it includes one file per
tool and configures `[vortac_manager]`.

For each tool, create/adjust a file under `vortac_configs/tools/`
(use `tools/tool0.cfg` as the template):

1. Copy `tools/tool0.cfg` → `tools/tool<N>.cfg`
2. Replace every `T0` / `tool0` / `dock0` with `T<N>` / `tool<N>` / `dock<N>`
3. Bump `tool_index`, set the board's `canbus_uuid` (from step 1)
4. Set placeholder `params_dock<N>_x/y/z` dock positions (calibrated later)
5. Uncomment/add `[include tools/tool<N>.cfg]` in `tools.cfg`

See [configs/README.md](configs/README.md) for details on the `include_with`
template mechanism and section renaming.

Restart Klipper after config changes:

```bash
sudo systemctl restart klipper
```

## 4. Calibrate the grabber (key)

Prerequisites: `[vortac_grabber]` loads without errors, the grabber can
rotate freely (no tool attached).

1. **Build the angle lookup table:**

   ```
   VORTAC_CALIBRATE
   ```

   The grabber turns forward through 2 full revolutions sampling 180 bins.
   At each bin it settles, takes 8 direct SPI reads of the AS5047D and
   stores their circular mean; the last revolution wins so backlash effects
   drop out. Optional parameters: `SAMPLES`, `SPEED`, `TURNS`, `PHASE`,
   `SETTLE`, `READS`, `READ_DWELL`.

2. On success Klipper reports `Calibration OK. lookup_table saved (...)` —
   persist it:

   ```
   SAVE_CONFIG
   ```

3. **Set the zero position:** move the grabber to its mechanical reference
   position <!-- TODO: document how the zero position is physically
   defined/approached -->, then:

   ```
   VORTAC_SET_ZERO
   SAVE_CONFIG
   ```

4. **Verify:**

   ```
   VORTAC_SIMPLE_READ      ; prints raw and true angle
   VORTAC_MOVE TARGET=90   ; closed-loop test move
   ```

   The true angle should read ~0° at the reference position and
   `VORTAC_MOVE` should reach its target within tolerance (default ±2°).
   `VORTAC_MESURE` sweeps a full revolution for detailed diagnostics.

## 5. Test the setup

1. **Grabber only** (no tool attached):

   ```
   VORTAC_SIMPLE_READ        ; sensor sanity check — raw and true angle
   VORTAC_ENGAGE             ; move to engage_pos (default 130)
   VORTAC_DISENGAGE          ; move to disengage_pos (default 0)
   ```

   Both are closed-loop moves to the positions configured on
   `[vortac_grabber]` — equivalent to `VORTAC_MOVE TARGET=<pos>`.

   Both moves should converge within tolerance (default ±2°).

   To verify the LUT linearizes the full 360°, run a diagnostic sweep:

   ```
   VORTAC_MESURE SAMPLES=180
   ```

   It steps through one full revolution and prints three data sets:
   `rawPairs` (commanded vs. raw sensor), `lutPairs` (LUT-corrected) and
   `finalPairs` (LUT + zero offset). Paste a set into
   `dev_scripts/plotData.py` to plot it: `rawPairs` may be arbitrarily
   non-linear, but `lutPairs`/`finalPairs` must form a straight line with
   slope 1 (one clean wrap at the 360° seam). The sweep starts at the
   current position, so the line is offset — the slope is what matters.

2. **Sense pins and dock detection** (tools parked in their docks):

   ```
   VORTAC_SENSE_STATUS       ; raw dock/grab sense pin states per tool
   VORTAC_DETECT             ; strobe-based dock occupancy + grabbed tool
   VORTAC_STATUS             ; current tool, dock map, QGL state
   ```

3. **First manual tool change** (after dock positions are calibrated):

   ```
   VORTAC_LOAD TOOL=T0       ; fetch T0 from its dock
   VORTAC_UNLOAD             ; park it back
   T0                        ; full tool-change command
   ```

<!-- TODO: continue guide — dock position calibration
     (VORTAC_SELECT_DOCK / VORTAC_DOCK_CAL_SAVE / VORTAC_DOCK_SAVE_POS)
     belongs before the first tool change test -->
