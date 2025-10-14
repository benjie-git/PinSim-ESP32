#!/bin/bash

TOOL_DIR=`dirname -- "$0"`/data
TOOL_DIR=`readlink -f -- $TOOL_DIR`

$TOOL_DIR/esptool --chip esp32s3 --baud 921600 --before default_reset --after hard_reset write_flash -e -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x0000 $TOOL_DIR/bootloader.bin 0x8000 $TOOL_DIR/partitions.bin 0xe000 $TOOL_DIR/boot_app0.bin 0x10000 $TOOL_DIR/firmware.bin
