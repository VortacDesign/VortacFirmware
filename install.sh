#!/bin/bash

# Paths for Klipper Python modules
KLIPPER_DIR=~/klipper
TARGET_PY="$KLIPPER_DIR/klippy/extras"
SOURCE_PY="$(pwd)/klipper-scripts"

# Paths for Klipper configuration files
CONFIG_TARGET=~/printer_data/config/vortac
CONFIG_SOURCE="$(pwd)/klipper-configs"

echo "🔧 Installing Vortac custom modules and configuration..."

# Ensure target directories exist
mkdir -p "$TARGET_PY"
mkdir -p "$CONFIG_TARGET"

# Copy Python modules
for file in "$SOURCE_PY"/*.py; do
    echo "➡️  Copying $(basename "$file") to $TARGET_PY"
    cp "$file" "$TARGET_PY"
done

# Copy configuration files
echo "🛠  Copying configuration files..."
for file in "$CONFIG_SOURCE"/*.cfg; do
    echo "➡️  Copying $(basename "$file") to $CONFIG_TARGET"
    cp "$file" "$CONFIG_TARGET"
done

echo "✅ Installation complete. Please restart Klipper and review your printer.cfg if needed."
