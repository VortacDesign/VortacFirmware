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
echo "🔧 Panels:   $REPO_DIR/klipperscreen/panels  →  \$HOME/KlipperScreen/panels (if present)"

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

# --- 1b) link KlipperScreen panels (only if KlipperScreen is installed) ---
KSCREEN_DIR="${KSCREEN_DIR:-$TARGET_HOME/KlipperScreen}"
PANELS_SRC="$REPO_DIR/klipperscreen/panels"
if [[ -d "$KSCREEN_DIR/panels" && -d "$PANELS_SRC" ]]; then
  echo "🔗 Linking KlipperScreen panels into $KSCREEN_DIR/panels"
  for f in "$PANELS_SRC"/*.py; do
    base="$(basename "$f")"
    dst="$KSCREEN_DIR/panels/$base"
    if [[ -e "$dst" && ! -L "$dst" ]]; then
      echo "  ⚠️  skip: $base (exists and is not a symlink)"
      continue
    fi
    ln -sfn "$f" "$dst"
    echo "  → $base"
  done
  # menu icon: KlipperScreen resolves `icon:` names against the active
  # theme's images dir, so link the logo into every theme
  ICON_SRC="$REPO_DIR/klipperscreen/icons/vortac_logo.svg"
  if [[ -f "$ICON_SRC" ]]; then
    for imgdir in "$KSCREEN_DIR"/styles/*/images; do
      [[ -d "$imgdir" ]] && ln -sfn "$ICON_SRC" "$imgdir/vortac_logo.svg"
    done
    echo "  → vortac_logo.svg (all themes)"
  fi
  echo "  (restart the UI to load them:  sudo systemctl restart KlipperScreen)"
else
  echo "ℹ️  KlipperScreen not found at $KSCREEN_DIR — skipping panel links"
fi

# --- 1c) serve the Vortac web UI via nginx (only if nginx is installed) ---
# Own server block on its own port — never touches the Mainsail site config.
VORTAC_UI_PORT="${VORTAC_UI_PORT:-7130}"
WEB_SRC="$REPO_DIR/web"
if command -v nginx >/dev/null 2>&1 && [[ -d "$WEB_SRC" ]]; then
  echo "🌐 Setting up nginx site for the Vortac web UI (port $VORTAC_UI_PORT)"
  NGINX_SITE="server {
    listen $VORTAC_UI_PORT;
    listen [::]:$VORTAC_UI_PORT;
    server_name _;
    root $WEB_SRC;
    index index.html;
    location / { try_files \$uri \$uri/ =404; }
}"
  if [[ -d /etc/nginx/sites-available && -d /etc/nginx/sites-enabled ]]; then
    echo "$NGINX_SITE" > /etc/nginx/sites-available/vortac-ui
    ln -sfn /etc/nginx/sites-available/vortac-ui /etc/nginx/sites-enabled/vortac-ui
  else
    echo "$NGINX_SITE" > /etc/nginx/conf.d/vortac-ui.conf
  fi
  if nginx -t >/dev/null 2>&1; then
    systemctl reload nginx 2>/dev/null || true
    echo "  → http://<pi>:$VORTAC_UI_PORT/"
  else
    echo "  ⚠️  nginx config test failed — site written but not reloaded (check: nginx -t)"
  fi
else
  echo "ℹ️  nginx not found — skipping Vortac web UI site"
fi

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
