#!/bin/bash

TOOL_DIR=PinSim_FW_updater/data/
CUR_DATE=$(date +"%Y%m%d")

mkdir dist

# NimBLE must be patched (MAX_BONDS > MAX_ADDRESSES) before building, otherwise
# the compile-time guard in src/xInput.cpp / src/keyboard.cpp fails the build.
./patch_nimble.sh

venv/bin/pio run -e release_PCB3
cp .pio/build/release_PCB3/bootloader.bin $TOOL_DIR/
cp .pio/build/release_PCB3/partitions.bin $TOOL_DIR/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin $TOOL_DIR/
cp .pio/build/release_PCB3/firmware.bin $TOOL_DIR/
mv PinSim_FW_updater PinSim_FW_updater_PCB3_$CUR_DATE
zip -r dist/PinSim_FW_updater_PCB3_$CUR_DATE.zip PinSim_FW_updater_PCB3_$CUR_DATE
cd PinSim_FW_updater_PCB3_$CUR_DATE/data
zip -r ../../dist/PinSim_FW_data_PCB3_$CUR_DATE.zip *.bin
cd ../..
mv PinSim_FW_updater_PCB3_$CUR_DATE PinSim_FW_updater

venv/bin/pio run -e release_PCB5
cp .pio/build/release_PCB5/bootloader.bin $TOOL_DIR/
cp .pio/build/release_PCB5/partitions.bin $TOOL_DIR/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin $TOOL_DIR/
cp .pio/build/release_PCB5/firmware.bin $TOOL_DIR/
mv PinSim_FW_updater PinSim_FW_updater_PCB5_$CUR_DATE
zip -r dist/PinSim_FW_updater_PCB5_$CUR_DATE.zip PinSim_FW_updater_PCB5_$CUR_DATE
cd PinSim_FW_updater_PCB5_$CUR_DATE/data
zip -r ../../dist/PinSim_FW_data_PCB5_$CUR_DATE.zip *.bin
cd ../..
mv PinSim_FW_updater_PCB5_$CUR_DATE PinSim_FW_updater

file="src/command.h"
version=$(sed -nE 's/^[[:space:]]*#[[:space:]]*define[[:space:]]+COMMAND_FW_VERSION[[:space:]]+"([^"]+)".*/\1/p' "$file")

# Upload to fw server
echo "../PinSim-fw-server/.venv/bin/python ../PinSim-fw-server/uploader.py \
$version \
dist/PinSim_FW_data_PCB3_$CUR_DATE.zip \
dist/PinSim_FW_updater_PCB3_$CUR_DATE.zip \
dist/PinSim_FW_data_PCB5_$CUR_DATE.zip \
dist/PinSim_FW_updater_PCB5_$CUR_DATE.zip
"

../PinSim-fw-server/.venv/bin/python ../PinSim-fw-server/uploader.py \
$version \
dist/PinSim_FW_data_PCB3_$CUR_DATE.zip \
dist/PinSim_FW_updater_PCB3_$CUR_DATE.zip \
dist/PinSim_FW_data_PCB5_$CUR_DATE.zip \
dist/PinSim_FW_updater_PCB5_$CUR_DATE.zip
