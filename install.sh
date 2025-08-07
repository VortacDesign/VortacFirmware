#!/usr/bin/env bash
# ----------------------------------------------------------
# Vortac installer
# - Copies all addons from klipper-scripts/extras → $KLIPPER_DIR/klippy/extras
# - Keeps configs bind-mounted for persistence
# ----------------------------------------------------------
set -euo pipefail

# Absolute path to this repo (directory of this script)
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"

# Klipper install dir; override with: KLIPPER_DIR=/path ./install.sh
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"

# Sources
SCRIPTS_SRC="$REPO_DIR/klipper-scripts/extras"
CONFIG_SRC="${CONFIG_SRC:-$REPO_DIR/klipper-configs/vortac_configs}"

# Targets
SCRIPTS_DST="$KLIPPER_DIR/klippy/extras"
CONFIG_DST="${CONFIG_DST:-$HOME/printer_data/config/vortac_configs}"

echo "🔧 Syncing extras → $SCRIPTS_DST"
mkdir -p "$SCRIPTS_DST"

# Copy/overwrite addon scripts (no sudo needed if you own the target)
if [[ -w "$SCRIPTS_DST" ]]; then
  rsync -a --delete --exclude='__pycache__/' --exclude='*.pyc' "$SCRIPTS_SRC/" "$SCRIPTS_DST/"
else
  sudo rsync -a --delete --exclude='__pycache__/' --exclude='*.pyc' "$SCRIPTS_SRC/" "$SCRIPTS_DST/"
fi

echo "🔗 Ensuring bind-mount for configs"
sudo mkdir -p "$CONFIG_DST"
# Make sure you can edit your local config source
sudo chown -R "$USER":"$USER" "$CONFIG_SRC" || true

# Add persistent bind-mount to /etc/fstab if missing
FSTAB_LINE="$CONFIG_SRC $CONFIG_DST none bind 0 0"
if ! grep -qsF "$FSTAB_LINE" /etc/fstab; then
  echo "$FSTAB_LINE" | sudo tee -a /etc/fstab >/dev/null
fi

# Mount (or remount) configs
if mountpoint -q "$CONFIG_DST"; then
  sudo mount -o remount,bind "$CONFIG_DST"
else
  sudo mount "$CONFIG_DST"
fi

echo "✅ Done.
• Extras synced to: $SCRIPTS_DST
• Configs bind-mounted at: $CONFIG_DST (persistent via /etc/fstab)
Please restart Klipper."
