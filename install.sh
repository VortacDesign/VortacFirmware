#!/usr/bin/env bash
# ----------------------------------------------------------
# Vortac installer (safe)
# - Copies your addons from klipper-scripts/extras → $KLIPPER_DIR/klippy/extras
#   without deleting core files (default: only items matching 'vortac_*')
# - Keeps configs bind-mounted (persistent via /etc/fstab)
# ----------------------------------------------------------
set -euo pipefail

# Paths (override via env if needed)
REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
KLIPPER_DIR="${KLIPPER_DIR:-$HOME/klipper}"

# Addons
SRC="$REPO_DIR/klipper-scripts/extras"
DST="$KLIPPER_DIR/klippy/extras"
# Copy pattern: copy only your files/folders by default (safer).
# Set PATTERN='*.py' to copy all .py files, or PATTERN='*' for everything.
PATTERN="${PATTERN:-vortac_*}"
# Dry-run (0/1) to preview what would be copied
DRY_RUN="${DRY_RUN:-0}"

# Configs (bind-mount)
CONFIG_SRC="${CONFIG_SRC:-$REPO_DIR/klipper-configs/vortac_configs}"
CONFIG_DST="${CONFIG_DST:-$HOME/printer_data/config/vortac_configs}"

echo "🔧 Preparing…"
[[ -d "$SRC" ]] || { echo "❌ Source not found: $SRC"; exit 1; }
mkdir -p "$DST"

# Use sudo only if destination not writable
SUDO=""
[[ -w "$DST" ]] || SUDO="sudo"

# rsync setup: include only your files/folders, never delete on target
RSYNC_OPTS=(-a --exclude='__pycache__/' --exclude='*.pyc')
RSYNC_FILTERS=(--include='*/' --include="${PATTERN}" --include="${PATTERN}/**" --exclude='*')
[[ "$DRY_RUN" == "1" ]] && RSYNC_OPTS+=(-n -v)

echo "📥 Copying addons (${PATTERN}) from $SRC → $DST (no deletes)"
$SUDO rsync "${RSYNC_OPTS[@]}" "${RSYNC_FILTERS[@]}" "$SRC/" "$DST/"

# ---------------- Config bind-mount ----------------
echo "🔗 Ensuring bind-mount for configs: $CONFIG_SRC → $CONFIG_DST"
sudo mkdir -p "$CONFIG_SRC" "$CONFIG_DST"
# make sure you can edit your local config source
sudo chown -R "$USER:$USER" "$CONFIG_SRC" || true

# Add persistent bind-mount to /etc/fstab if missing
FSTAB_LINE="$CONFIG_SRC $CONFIG_DST none bind 0 0"
if ! grep -qsF "$FSTAB_LINE" /etc/fstab; then
  echo "$FSTAB_LINE" | sudo tee -a /etc/fstab >/dev/null
fi

# Mount (or remount) the configs
if mountpoint -q "$CONFIG_DST"; then
  sudo mount -o remount,bind "$CONFIG_DST"
else
  # prefer fstab-based mount; fallback to direct bind if needed
  sudo mount "$CONFIG_DST" || sudo mount --bind "$CONFIG_SRC" "$CONFIG_DST"
fi

echo "✅ Done.
• Addons copied to: $DST  (core files untouched)
• Configs bind-mounted at: $CONFIG_DST (persist via /etc/fstab)
→ Restart Klipper:  sudo systemctl restart klipper
