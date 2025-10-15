#!/bin/bash

TOOL_DIR=PinSim_FW_updater/data/
CUR_DATE=$(date +"%Y%m%d")

mkdir dist

pio run -e release_PCB3
cp .pio/build/devel/bootloader.bin $TOOL_DIR/
cp .pio/build/devel/partitions.bin $TOOL_DIR/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin $TOOL_DIR/
cp .pio/build/devel/firmware.bin $TOOL_DIR/
mv PinSim_FW_updater PinSim_FW_updater_PCB3_$CUR_DATE
zip -r dist/PinSim_FW_updater_PCB3_$CUR_DATE.zip PinSim_FW_updater_PCB3_$CUR_DATE
mv PinSim_FW_updater_PCB3_$CUR_DATE PinSim_FW_updater

pio run -e release_PCB5
cp .pio/build/devel/bootloader.bin $TOOL_DIR/
cp .pio/build/devel/partitions.bin $TOOL_DIR/
cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin $TOOL_DIR/
cp .pio/build/devel/firmware.bin $TOOL_DIR/
mv PinSim_FW_updater PinSim_FW_updater_PCB5_$CUR_DATE
zip -r dist/PinSim_FW_updater_PCB5_$CUR_DATE.zip PinSim_FW_updater_PCB5_$CUR_DATE
mv PinSim_FW_updater_PCB5_$CUR_DATE PinSim_FW_updater
