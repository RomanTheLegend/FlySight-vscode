#!/bin/bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
ELF="$REPO_ROOT/build/Release/FlySight_STM32_MMG_New.elf"
BIN_DIR="$REPO_ROOT/build/Binary"
BIN="$BIN_DIR/FlySight_STM32_MMG_New.bin"
DEPLOY_DIR="$REPO_ROOT/Deploy"

echo "=== Building ==="
cmake --build "$REPO_ROOT/build/Release" -- -j$(sysctl -n hw.logicalcpu)

echo "=== Converting to binary ==="
mkdir -p "$BIN_DIR"
arm-none-eabi-objcopy -O binary "$ELF" "$BIN"
echo "Binary: $(wc -c < "$BIN" | tr -d ' ') bytes"

echo "=== Signing ==="
cd "$DEPLOY_DIR"
source .venv/bin/activate
python3 deploy_firmware.py "$BIN"

echo ""
echo "=== Done ==="
echo "SFB files in $DEPLOY_DIR/Firmware_To_Deploy/"
ls -lh "$DEPLOY_DIR/Firmware_To_Deploy/"*.sfb
