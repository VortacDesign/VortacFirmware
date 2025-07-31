#!/usr/bin/env bash
# ---------------------------------------------
# Vortac Install Script (install.sh)
# - Ensures persistent bind-mount of your custom config folder
# - Ensures persistent bind-mount of your custom Python scripts folder
# ---------------------------------------------
set -euo pipefail

# Repository root (script location)
REPO_DIR="$(pwd)"
# Klipper installation directory (absolute path)
KLIPPER_DIR="$HOME/klipper"

# Paths for Configurations
CONFIG_SOURCE="$REPO_DIR/klipper-configs/vortac_configs"
CONFIG_TARGET="$HOME/printer_data/config/vortac_configs"

# Paths for Python Scripts
SCRIPTS_SOURCE="$REPO_DIR/klipper/klippy/extras/vortac_scripts#!/usr/bin/env bash
# ---------------------------------------------
# Vortac Install Script (install.sh)
# - Ensures persistent bind-mount of your custom config folder
# - Ensures persistent bind-mount of your custom Python scripts folder
# ---------------------------------------------
set -euo pipefail

# Repository root (script location)
REPO_DIR="$(pwd)"
# Klipper installation directory (absolute path)
KLIPPER_DIR="/home/pi/klipper"

# Paths for Configurations
CONFIG_SOURCE="$REPO_DIR/klipper-configs/vortac_configs"
CONFIG_TARGET="/home/pi/printer_data/config/vortac_configs"

# Paths for Python Scripts
SCRIPTS_SOURCE="$REPO_DIR/klipper/klippy/extras/vortac_scripts"
SCRIPTS_TARGET="$KLIPPER_DIR/klippy/extras/vortac_scripts"

# Helper for sudo (ensure NOPASSWD for mount and fstab edits)
SUDO="sudo"

echo "🔧 Installing Vortac custom modules and configuration..."

# 1) Ensure target directories exist
mkdir -p "$CONFIG_TARGET"
mkdir -p "$SCRIPTS_TARGET"

# 2) Fix ownership so 'pi' can read/write
echo "🔒 Setting ownership of source directories"
$SUDO chown -R pi:pi "$CONFIG_SOURCE"
$SUDO chown -R pi:pi "$SCRIPTS_SOURCE"

# 3) Persistently register bind-mounts in /etc/fstab if not present
FSTAB_CFG="$CONFIG_SOURCE $CONFIG_TARGET none bind 0 0"
FSTAB_SCR="$SCRIPTS_SOURCE $SCRIPTS_TARGET none bind 0 0"
if ! $SUDO grep -qF "$FSTAB_CFG" /etc/fstab; then
  echo "🔒 Adding config bind-mount to /etc/fstab"
  echo "$FSTAB_CFG" | $SUDO tee -a /etc/fstab
else
  echo "🔄 Config fstab entry already exists"
fi
if ! $SUDO grep -qF "$FSTAB_SCR" /etc/fstab; then
  echo "🔒 Adding scripts bind-mount to /etc/fstab"
  echo "$FSTAB_SCR" | $SUDO tee -a /etc/fstab
else
  echo "🔄 Scripts fstab entry already exists"
fi

# 4) Mount the config and scripts directories (idempotent)
if ! mountpoint -q "$CONFIG_TARGET"; then
  echo "🔗 Mounting configs: '$CONFIG_SOURCE' → '$CONFIG_TARGET'"
  $SUDO mount --bind "$CONFIG_SOURCE" "$CONFIG_TARGET"
else
  echo "🔄 '$CONFIG_TARGET' is already mounted"
fi
if ! mountpoint -q "$SCRIPTS_TARGET"; then
  echo "🔗 Mounting scripts: '$SCRIPTS_SOURCE' → '$SCRIPTS_TARGET'"
  $SUDO mount --bind "$SCRIPTS_SOURCE" "$SCRIPTS_TARGET"
else
  echo "🔄 '$SCRIPTS_TARGET' is already mounted"
fi

# 5) Summary
cat <<EOF
✅ Installation complete.
• Configs bound at '$CONFIG_TARGET' (persistent via fstab).
• Python scripts bound at '$SCRIPTS_TARGET' (persistent via fstab).

Please restart Klipper to apply changes.
EOF"
SCRIPTS_TARGET="$KLIPPER_DIR/klippy/extras/vortac_scripts"

# Helper for sudo (ensure NOPASSWD for mount and fstab edits)
SUDO="sudo"

echo "🔧 Installing Vortac custom modules and configuration..."

# 1) Ensure target directories exist
mkdir -p "$CONFIG_TARGET"
mkdir -p "$SCRIPTS_TARGET"

# 2) Fix ownership so 'pi' can read/write
echo "🔒 Setting ownership of source directories"
$SUDO chown -R pi:pi "$CONFIG_SOURCE"
$SUDO chown -R pi:pi "$SCRIPTS_SOURCE"

# 3) Persistently register bind-mounts in /etc/fstab if not present
FSTAB_CFG="$CONFIG_SOURCE $CONFIG_TARGET none bind 0 0"
FSTAB_SCR="$SCRIPTS_SOURCE $SCRIPTS_TARGET none bind 0 0"
if ! $SUDO grep -qF "$FSTAB_CFG" /etc/fstab; then
  echo "🔒 Adding config bind-mount to /etc/fstab"
  echo "$FSTAB_CFG" | $SUDO tee -a /etc/fstab
else
  echo "🔄 Config fstab entry already exists"
fi
if ! $SUDO grep -qF "$FSTAB_SCR" /etc/fstab; then
  echo "🔒 Adding scripts bind-mount to /etc/fstab"
  echo "$FSTAB_SCR" | $SUDO tee -a /etc/fstab
else
  echo "🔄 Scripts fstab entry already exists"
fi

# 4) Mount the config and scripts directories (idempotent)
if ! mountpoint -q "$CONFIG_TARGET"; then
  echo "🔗 Mounting configs: '$CONFIG_SOURCE' → '$CONFIG_TARGET'"
  $SUDO mount --bind "$CONFIG_SOURCE" "$CONFIG_TARGET"
else
  echo "🔄 '$CONFIG_TARGET' is already mounted"
fi
if ! mountpoint -q "$SCRIPTS_TARGET"; then
  echo "🔗 Mounting scripts: '$SCRIPTS_SOURCE' → '$SCRIPTS_TARGET'"
  $SUDO mount --bind "$SCRIPTS_SOURCE" "$SCRIPTS_TARGET"
else
  echo "🔄 '$SCRIPTS_TARGET' is already mounted"
fi

# 5) Summary
cat <<EOF
✅ Installation complete.
• Configs bound at '$CONFIG_TARGET' (persistent via fstab).
• Python scripts bound at '$SCRIPTS_TARGET' (persistent via fstab).

Please restart Klipper to apply changes.
EOF
