#!/usr/bin/env bash
# ---------------------------------------------
# Vortac Install Script (install.sh)
# - Ensures persistent bind-mount of your custom config folder
# - Deploys Python scripts into Klipper extras
# ---------------------------------------------
set -euo pipefail
# Paths for Klipper Python modules\ nKLIPPER_DIR="$HOME/klipper"
TARGET_PY="$HOME/klipper/klippy/extras"
SOURCE_PY="$(pwd)/klipper-scripts"

# Paths for Klipper configuration files
CONFIG_SOURCE="$(pwd)/klipper-configs/vortac_configs"
# Customize the target folder name as needed
CONFIG_TARGET="$HOME/printer_data/config/vortac_configs"

# Helper for sudo (ensure NOPASSWD for mount/editing fstab or run as root)
SUDO="sudo"

echo "🔧 Installing Vortac custom modules and configuration..."

# 1) Ensure target directories exist
mkdir -p "$TARGET_PY"
mkdir -p "$CONFIG_TARGET"

# 2) Persistently register bind-mount in /etc/fstab if not present
FSTAB_ENTRY="$CONFIG_SOURCE $CONFIG_TARGET none bind 0 0"
if ! $SUDO grep -qF "$CONFIG_SOURCE $CONFIG_TARGET none bind" /etc/fstab; then
  echo "🔒 Adding bind-mount to /etc/fstab"
  echo "$FSTAB_ENTRY" | $SUDO tee -a /etc/fstab
else
  echo "🔄 fstab entry for bind-mount already exists"
fi

# 3) Bind-mount the config directory (idempotent)
if ! mountpoint -q "$CONFIG_TARGET"; then
  echo "🔗 Mounting '$CONFIG_SOURCE' → '$CONFIG_TARGET'"
  $SUDO mount --bind "$CONFIG_SOURCE" "$CONFIG_TARGET"
else
  echo "🔄 '$CONFIG_TARGET' is already mounted"
fi

# 4) Deploy Python scripts into Klipper extras
echo "🛠  Deploying Python scripts to Klipper extras..."
for file in "$SOURCE_PY"/*.py; do
  echo "➡️  Copying $(basename "$file") to $TARGET_PY"
  cp -u "$file" "$TARGET_PY"
  $SUDO chmod 644 "$TARGET_PY/$(basename "$file")"
done

# 5) Summary
cat <<EOF
✅ Installation complete.
• Configs bound at '$CONFIG_TARGET' (persistent via fstab).
• Python scripts copied to '$TARGET_PY'.

Please restart Klipper to apply changes.
EOF
