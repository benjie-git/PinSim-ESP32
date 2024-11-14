#!/bin/bash

data/esptool --chip esp32s3 --baud 921600 --before default_reset --after hard_reset write_flash  -z --flash_mode keep --flash_freq keep --flash_size keep 0x0 data/PinSim-ESP32.ino.bootloader.bin 0x8000 data/PinSim-ESP32.ino.partitions.bin 0xe000 data/boot_app0.bin 0x10000 data/PinSim-ESP32.ino.bin
