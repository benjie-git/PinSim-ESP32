#!/bin/bash

TOOL_DIR=PinSim_FW_updater/data/
CUR_DATE=$(date +"%Y%m%d")

mkdir dist

venv/bin/pio run -e release_PCB3
cp .pio/build/release_PCB3/bootloader.bin $TOOL_DIR/
cp .pio/build/release_PCB3/partitions.bin $TOOL_DIR/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin $TOOL_DIR/
cp .pio/build/release_PCB3/firmware.bin $TOOL_DIR/
mv PinSim_FW_updater PinSim_FW_updater_PCB3_$CUR_DATE
zip -r dist/PinSim_FW_updater_PCB3_$CUR_DATE.zip PinSim_FW_updater_PCB3_$CUR_DATE
mv PinSim_FW_updater_PCB3_$CUR_DATE PinSim_FW_updater

venv/bin/pio run -e release_PCB5
cp .pio/build/release_PCB5/bootloader.bin $TOOL_DIR/
cp .pio/build/release_PCB5/partitions.bin $TOOL_DIR/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin $TOOL_DIR/
cp .pio/build/release_PCB5/firmware.bin $TOOL_DIR/
mv PinSim_FW_updater PinSim_FW_updater_PCB5_$CUR_DATE
zip -r dist/PinSim_FW_updater_PCB5_$CUR_DATE.zip PinSim_FW_updater_PCB5_$CUR_DATE
mv PinSim_FW_updater_PCB5_$CUR_DATE PinSim_FW_updater

scp dist/PinSim_FW_updater_PCB3_$CUR_DATE.zip dist/PinSim_FW_updater_PCB5_$CUR_DATE.zip $1:www/files/
echo "https://$2/files/PinSim_FW_updater_PCB3_$CUR_DATE.zip"
echo "https://$2/files/PinSim_FW_updater_PCB5_$CUR_DATE.zip"
