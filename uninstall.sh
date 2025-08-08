#!/usr/bin/env bash
# ---------------------------------------------
# Vortac Uninstall Script (uninstall.sh)
# - Removes persistent bind-mount from /etc/fstab
# - Unmounts the config directory
# ---------------------------------------------
set -euo pipefail

# Paths (must match your install.sh)
CONFIG_SOURCE="$(pwd)/klipper-configs"
CONFIG_TARGET="~/printer_data/config/vortac_configs"

# Helper for sudo (ensure NOPASSWD for mount/editing fstab or run as root)
SUDO="sudo"

echo "🗑️  Uninstalling Vortac bind-mount and cleaning up..."

# 1) Unmount the bind-mount if active
if mountpoint -q "$CONFIG_TARGET"; then
  echo "⬇️  Unmounting '$CONFIG_TARGET'"
  $SUDO umount "$CONFIG_TARGET"
else
  echo "ℹ️  '$CONFIG_TARGET' is not mounted"
fi

# 2) Remove the entry from /etc/fstab
echo "🧹  Removing bind-mount entry from /etc/fstab"
# Backup fstab before modifying
$SUDO cp /etc/fstab /etc/fstab.bak
# Filter out the matching line
$SUDO grep -vF "$CONFIG_SOURCE $CONFIG_TARGET none bind" /etc/fstab.bak | $SUDO tee /etc/fstab > /dev/null

echo "✅ Uninstall complete."
exit 0
