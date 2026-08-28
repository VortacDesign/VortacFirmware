#!/usr/bin/env bash
# ----------------------------------------------------------
# Vortac uninstaller
# - Disables & removes systemd watcher and mount units
# - Unmounts config bind-mount
# - Deletes ONLY our addon symlinks in Klipper's extras
# - Leaves your repo and source configs intact
# ----------------------------------------------------------
set -euo pipefail

# --- need root for systemd + unmount ---
if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo:  sudo $0"; exit 1
fi

# --- resolve target user/home (works when run via sudo) ---
TARGET_USER="${TARGET_USER:-${SUDO_USER:-$(id -un)}}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
[[ -n "$TARGET_HOME" ]] || { echo "❌ Cannot resolve home for '$TARGET_USER'"; exit 1; }

# --- paths (match the installer) ---
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
KLIPPER_DIR="${KLIPPER_DIR:-$TARGET_HOME/klipper}"

SCRIPTS_SRC="$REPO_DIR/klippy/extras"
SCRIPTS_DST="$KLIPPER_DIR/klippy/extras"

CONFIG_SRC="$REPO_DIR/configs/vortac_configs"               # info only (we do NOT touch it)
CONFIG_DST="$TARGET_HOME/printer_data/config/vortac_configs"

# systemd unit names/paths
SERVICE_NAME="vortac-install"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
PATH_FILE="/etc/systemd/system/${SERVICE_NAME}.path"

MOUNT_UNIT="$(systemd-escape --path --suffix=mount "$CONFIG_DST")"
AUTOMOUNT_UNIT="$(systemd-escape --path --suffix=automount "$CONFIG_DST")"
MOUNT_FILE="/etc/systemd/system/$MOUNT_UNIT"
AUTOMOUNT_FILE="/etc/systemd/system/$AUTOMOUNT_UNIT"

echo "🧹 Uninstalling Vortac:"
echo "    user:   $TARGET_USER ($TARGET_HOME)"
echo "    extras: $SCRIPTS_DST  (source: $SCRIPTS_SRC)"
echo "    config: $CONFIG_DST   (source: $CONFIG_SRC)"

# --- 1) Stop watcher & mount units (best effort) ---
systemctl disable --now "${SERVICE_NAME}.path" 2>/dev/null || true
systemctl stop "${SERVICE_NAME}.service" 2>/dev/null || true

systemctl disable --now "$AUTOMOUNT_UNIT" 2>/dev/null || true
systemctl stop "$MOUNT_UNIT" 2>/dev/null || true

# --- 2) Unmount configs (try normal, then lazy if busy) ---
if mountpoint -q "$CONFIG_DST"; then
  echo "🔻 Unmounting $CONFIG_DST"
  # try to stop Klipper to free handles (best effort)
  if systemctl is-active --quiet klipper; then
    systemctl stop klipper || true
  fi
  umount "$CONFIG_DST" 2>/dev/null || umount -l "$CONFIG_DST" 2>/dev/null || true
fi

# --- 3) Remove systemd unit files & reload ---
rm -f "$SERVICE_FILE" "$PATH_FILE" "$AUTOMOUNT_FILE" "$MOUNT_FILE"
systemctl daemon-reload

# --- 4) Legacy cleanup: remove any old fstab bind entry for CONFIG_DST (if present) ---
if grep -qs "$CONFIG_DST" /etc/fstab; then
  echo "🧾 Cleaning legacy /etc/fstab entry for $CONFIG_DST"
  cp /etc/fstab "/etc/fstab.bak.$(date +%Y%m%d_%H%M%S)"
  # delete lines that mention the exact target path and 'bind'
  sed -i "\|$CONFIG_DST.*bind|d" /etc/fstab
fi

# --- 5) Remove ONLY our symlinks in Klipper extras ---
if [[ -d "$SCRIPTS_DST" ]]; then
  echo "🧷 Removing addon symlinks in $SCRIPTS_DST"
  # primary: links pointing into our repo
  find "$SCRIPTS_DST" -maxdepth 1 -type l -lname "$SCRIPTS_SRC/*" -print -delete || true
  # fallback: also remove links matching basenames of our source .py files (if repo moved)
  while IFS= read -r -d '' f; do
    base="$(basename "$f")"
    [[ -L "$SCRIPTS_DST/$base" ]] && { echo "  → $base"; rm -f "$SCRIPTS_DST/$base"; }
  done < <(find "$SCRIPTS_SRC" -maxdepth 1 -type f -name "*.py" -print0 2>/dev/null || true)
fi

# --- 5b) Remove ONLY our KlipperScreen panel symlinks ---
KSCREEN_DIR="${KSCREEN_DIR:-$TARGET_HOME/KlipperScreen}"
PANELS_SRC="$REPO_DIR/klipperscreen/panels"
if [[ -d "$KSCREEN_DIR/panels" ]]; then
  echo "🧷 Removing panel symlinks in $KSCREEN_DIR/panels"
  find "$KSCREEN_DIR/panels" -maxdepth 1 -type l -lname "$PANELS_SRC/*" -print -delete || true
fi
for imgdir in "$KSCREEN_DIR"/styles/*/images; do
  [[ -d "$imgdir" ]] && find "$imgdir" -maxdepth 1 -type l -lname "$REPO_DIR/klipperscreen/icons/*" -print -delete || true
done

# --- 5c) Remove the Vortac web UI nginx site ---
rm -f /etc/nginx/sites-enabled/vortac-ui /etc/nginx/sites-available/vortac-ui \
      /etc/nginx/conf.d/vortac-ui.conf
if command -v nginx >/dev/null 2>&1 && nginx -t >/dev/null 2>&1; then
  systemctl reload nginx 2>/dev/null || true
fi

# --- 6) Remove empty mountpoint dir (optional) ---
rmdir "$CONFIG_DST" 2>/dev/null || true

echo "✅ Uninstall complete."
echo "   (Repo and source configs untouched: $REPO_DIR)"
echo "   You can start Klipper again if needed:  sudo systemctl start klipper"
