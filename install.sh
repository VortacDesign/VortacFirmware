#!/usr/bin/env bash
# ----------------------------------------------------------
# Vortac installer (simple & production)
# - Links all .py from repo's klippy/extras -> $KLIPPER_DIR/klippy/extras
# - Bind-mounts configs (persistent via systemd .mount/.automount)
# - Re-runs itself after every repo update (systemd .path)
# ----------------------------------------------------------
set -euo pipefail

# --- must run as root (write systemd units + mount) ---
if [[ $EUID -ne 0 ]]; then
  echo "Please run with sudo:  sudo $0"; exit 1
fi

# --- detect target user/home (works with sudo) ---
TARGET_USER="${TARGET_USER:-${SUDO_USER:-$(id -un)}}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
[[ -n "$TARGET_HOME" ]] || { echo "❌ Cannot resolve home for '$TARGET_USER'"; exit 1; }

# --- paths (your layout) ---
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
KLIPPER_DIR="${KLIPPER_DIR:-$TARGET_HOME/klipper}"

SCRIPTS_SRC="$REPO_DIR/klippy/extras"
SCRIPTS_DST="$KLIPPER_DIR/klippy/extras"

CONFIG_SRC="$REPO_DIR/configs/vortac_configs"
CONFIG_DST="$TARGET_HOME/printer_data/config/vortac_configs"

# systemd unit names/paths
SERVICE_NAME="vortac-install"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
PATH_FILE="/etc/systemd/system/${SERVICE_NAME}.path"

MOUNT_UNIT="$(systemd-escape --path --suffix=mount "$CONFIG_DST")"
AUTOMOUNT_UNIT="$(systemd-escape --path --suffix=automount "$CONFIG_DST")"
MOUNT_FILE="/etc/systemd/system/$MOUNT_UNIT"
AUTOMOUNT_FILE="/etc/systemd/system/$AUTOMOUNT_UNIT"

# figure out the branch to watch (fallback)
BRANCH="$(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
[[ "$BRANCH" == "HEAD" || -z "$BRANCH" ]] && BRANCH="development"

echo "🔧 Repo:     $REPO_DIR"
echo "🔧 Target:   $TARGET_USER ($TARGET_HOME)"
echo "🔧 Klipper:  $KLIPPER_DIR"
echo "🔧 Branch:   $BRANCH"
echo "🔧 Addons:   $SCRIPTS_SRC  →  $SCRIPTS_DST (link all .py)"
echo "🔧 Configs:  bind-mount:   $CONFIG_SRC  →  $CONFIG_DST"

# --- sanity checks (no creation of SCRIPTS_DST per your wish) ---
[[ -d "$SCRIPTS_SRC" ]] || { echo "❌ Missing $SCRIPTS_SRC"; exit 1; }
[[ -d "$SCRIPTS_DST" ]] || { echo "❌ Missing $SCRIPTS_DST (Klipper not installed?)"; exit 1; }
[[ -d "$CONFIG_SRC"  ]] || { echo "❌ Missing $CONFIG_SRC"; exit 1; }

# --- 1) link addons (top-level .py; no recursion; no mkdirs for subdirs) ---
echo "🔗 Linking addons into $SCRIPTS_DST"
shopt -s nullglob
for f in "$SCRIPTS_SRC"/*.py; do
  base="$(basename "$f")"
  dst="$SCRIPTS_DST/$base"
  if [[ -e "$dst" && ! -L "$dst" ]]; then
    echo "  ⚠️  skip: $base (exists and is not a symlink)"
    continue
  fi
  ln -sfn "$f" "$dst"
  echo "  → $base"
done

# --- 2) bind-mount configs via systemd (persistent, no fstab) ---
echo "🪢 Setting up systemd bind-mount for configs"
# mountpoint must exist; source bleibt unangetastet
mkdir -p "$CONFIG_DST"

# .mount unit
cat > "$MOUNT_FILE" <<EOF
[Unit]
Description=Vortac configs bind mount
After=local-fs.target

[Mount]
What=$CONFIG_SRC
Where=$CONFIG_DST
Type=none
Options=bind

[Install]
WantedBy=multi-user.target
EOF

# .automount unit (on-demand)
cat > "$AUTOMOUNT_FILE" <<EOF
[Unit]
Description=Automount Vortac configs

[Automount]
Where=$CONFIG_DST

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now "$AUTOMOUNT_UNIT"
# nicht explizit .mount starten (vermeidet Race); Zugriff triggert Mount:
stat "$CONFIG_DST" >/dev/null 2>&1 || true

# --- 3) watcher: rerun this installer after each repo update ---
echo "🛠  Installing repo update watcher"
cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=Run Vortac installer after repo updates
After=network-online.target

[Service]
Type=oneshot
User=root
WorkingDirectory=$REPO_DIR
Environment=TARGET_USER=$TARGET_USER
Environment=KLIPPER_DIR=$KLIPPER_DIR
ExecStart=/bin/bash -lc '$REPO_DIR/install.sh'
EOF

cat > "$PATH_FILE" <<EOF
[Unit]
Description=Watch Vortac repo for updates

[Path]
PathChanged=$REPO_DIR/.git/refs/heads/$BRANCH
PathChanged=$REPO_DIR/.git/packed-refs

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now "${SERVICE_NAME}.path"

echo "✅ Done. Now restart Klipper:  sudo systemctl restart klipper"
