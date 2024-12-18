#!/bin/bash

TOOL_DIR=`dirname -- "$0"`/data
TOOL_DIR=`readlink -f -- $TOOL_DIR`
$TOOL_DIR/esptool --chip esp32s3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode keep --flash_freq keep --flash_size keep 0x0 $TOOL_DIR/PinSim-ESP32.ino.bootloader.bin 0x8000 $TOOL_DIR/PinSim-ESP32.ino.partitions.bin 0xe000 $TOOL_DIR/boot_app0.bin 0x10000 $TOOL_DIR/PinSim-ESP32.ino.bin
