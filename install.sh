#!/usr/bin/env bash
# ----------------------------------------------------------
# Vortac installer (symlink-first + auto-run on repo updates)
# Repo layout:
#   configs/vortac_configs/      -> symlinked to ~/printer_data/config/vortac_configs  (default)
#   klippy/extras/*.py           -> symlinked into $KLIPPER_DIR/klippy/extras
#
# Env knobs:
#   KLIPPER_DIR=/home/pi/klipper
#   PATTERN='*.py'            # which addon files to link from klippy/extras
#   CONFIG_MODE=mount         # set to "mount" to use bind-mount for configs instead of symlink
#   TARGET_USER=pi            # user for systemd service (defaults to $USER)
# ----------------------------------------------------------
set -euo pipefail

# --- Paths ---
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"

SCRIPTS_SRC="$REPO_DIR/klippy/extras"
SCRIPTS_DST="$KLIPPER_DIR/klippy/extras"
PATTERN="${PATTERN:-*.py}"

CONFIG_SRC="$REPO_DIR/configs/vortac_configs"
CONFIG_DST="$HOME/printer_data/config/vortac_configs"
CONFIG_MODE="${CONFIG_MODE:-symlink}"   # symlink | mount

TARGET_USER="${TARGET_USER:-$USER}"

SERVICE_NAME="vortac-install"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
PATH_FILE="/etc/systemd/system/${SERVICE_NAME}.path"

# Detect current branch (for the watcher)
BRANCH="$(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo development)"

echo "🔧 Repo:     $REPO_DIR"
echo "🔧 KLIPPER:  $KLIPPER_DIR"
echo "🔧 Branch:   $BRANCH"
echo "🔧 Pattern:  $PATTERN"
echo "🔧 Configs:  $CONFIG_MODE ($CONFIG_SRC → $CONFIG_DST)"

# --- sanity checks ---
[[ -d "$SCRIPTS_SRC" ]] || { echo "❌ Missing $SCRIPTS_SRC"; exit 1; }
[[ -d "$KLIPPER_DIR/klippy" ]] || { echo "❌ KLIPPER_DIR seems wrong: $KLIPPER_DIR (no klippy/)"; exit 1; }

# --- link addons (per-file symlinks) ---
echo "🔗 Linking addons into $SCRIPTS_DST"
mkdir -p "$SCRIPTS_DST"
shopt -s nullglob
COUNT=0
# link only files matching PATTERN at top-level of extras (adjust find if you have subfolders)
for f in "$SCRIPTS_SRC"/$PATTERN; do
  base="$(basename "$f")"
  ln -sfn "$f" "$SCRIPTS_DST/$base"
  COUNT=$((COUNT+1))
done
echo "   → linked $COUNT file(s)."

# --- configs: symlink (default) or bind-mount (fallback) ---
if [[ "$CONFIG_MODE" == "symlink" ]]; then
  echo "🔗 Symlinking configs: $CONFIG_DST → $CONFIG_SRC"
  mkdir -p "$(dirname "$CONFIG_DST")"
  ln -sfn "$CONFIG_SRC" "$CONFIG_DST"
else
  echo "🪢 Bind-mounting configs (requires sudo)"
  sudo mkdir -p "$CONFIG_SRC" "$CONFIG_DST"
  sudo chown -R "$TARGET_USER":"$TARGET_USER" "$CONFIG_SRC" || true
  FSTAB_LINE="$CONFIG_SRC $CONFIG_DST none bind 0 0"
  if ! grep -qsF "$FSTAB_LINE" /etc/fstab; then
    echo "$FSTAB_LINE" | sudo tee -a /etc/fstab >/dev/null
  fi
  if mountpoint -q "$CONFIG_DST"; then
    sudo mount -o remount,bind "$CONFIG_DST"
  else
    sudo mount "$CONFIG_DST" || sudo mount --bind "$CONFIG_SRC" "$CONFIG_DST"
  fi
fi

# --- systemd path trigger (auto-run this script after repo updates) ---
echo "🛠  Installing systemd watcher"
sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=Run Vortac installer after repo updates
After=network-online.target

[Service]
Type=oneshot
User=$TARGET_USER
WorkingDirectory=$REPO_DIR
ExecStart=/bin/bash -lc '$REPO_DIR/install.sh'
EOF

sudo tee "$PATH_FILE" >/dev/null <<EOF
[Unit]
Description=Watch Vortac repo for updates

[Path]
PathChanged=$REPO_DIR/.git/refs/heads/$BRANCH
PathChanged=$REPO_DIR/.git/packed-refs

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}.path"

cat <<'MSG'
✅ Setup complete.
• Addons are symlinked into Klipper's extras.
• Configs are symlinked (or bind-mounted if CONFIG_MODE=mount).
• Auto-installer is active and will re-run after each repo update.
→ Restart Klipper when convenient:  sudo systemctl restart klipper
MSG
