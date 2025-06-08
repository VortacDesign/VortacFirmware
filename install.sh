#!/bin/bash

KLIPPER_DIR=~/klipper
TARGET_DIR="$KLIPPER_DIR/klippy/extras"
SOURCE_DIR="$(pwd)/klipper-scripts"

echo "🔧 Installing Vortac custom modules..."

for file in "$SOURCE_DIR"/*.py; do
    echo "➡️  Copying $(basename "$file") to $TARGET_DIR"
    cp "$file" "$TARGET_DIR"
done

echo "✅ Installation complete. Please restart Klipper."
