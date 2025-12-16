#!/bin/bash

sed -i .bak 's/\/\/ #define CONFIG_BT_NIMBLE_MAX_BONDS 3/#undef CONFIG_BT_NIMBLE_MAX_BONDS\n#define CONFIG_BT_NIMBLE_MAX_BONDS 5/' .pio/libdeps/devel/NimBLE-Arduino/src/nimconfig.h
sed -i .bak 's/\/\/ #define CONFIG_BT_NIMBLE_MAX_BONDS 3/#undef CONFIG_BT_NIMBLE_MAX_BONDS\n#define CONFIG_BT_NIMBLE_MAX_BONDS 5/' .pio/libdeps/release_PCB3/NimBLE-Arduino/src/nimconfig.h
sed -i .bak 's/\/\/ #define CONFIG_BT_NIMBLE_MAX_BONDS 3/#undef CONFIG_BT_NIMBLE_MAX_BONDS\n#define CONFIG_BT_NIMBLE_MAX_BONDS 5/' .pio/libdeps/release_PCB5/NimBLE-Arduino/src/nimconfig.h

sed -i .bak 's/\/\/ #define CONFIG_BT_NIMBLE_MAX_CCCDS 8/#undef CONFIG_BT_NIMBLE_MAX_CCCDS\n#define CONFIG_BT_NIMBLE_MAX_CCCDS 20/' .pio/libdeps/devel/NimBLE-Arduino/src/nimconfig.h
sed -i .bak 's/\/\/ #define CONFIG_BT_NIMBLE_MAX_CCCDS 8/#undef CONFIG_BT_NIMBLE_MAX_CCCDS\n#define CONFIG_BT_NIMBLE_MAX_CCCDS 20/' .pio/libdeps/release_PCB3/NimBLE-Arduino/src/nimconfig.h
sed -i .bak 's/\/\/ #define CONFIG_BT_NIMBLE_MAX_CCCDS 8/#undef CONFIG_BT_NIMBLE_MAX_CCCDS\n#define CONFIG_BT_NIMBLE_MAX_CCCDS 20/' .pio/libdeps/release_PCB5/NimBLE-Arduino/src/nimconfig.h

sed -i .bak 's/#define NIMBLE_NVS_NAMESPACE                     "nimble_bond"/const char* NIMBLE_NVS_NAMESPACE               = "nimble_bond";/' .pio/libdeps/devel/NimBLE-Arduino/src/nimble/nimble/host/store/config/src/ble_store_nvs.c
sed -i .bak 's/#define NIMBLE_NVS_NAMESPACE                     "nimble_bond"/const char* NIMBLE_NVS_NAMESPACE               = "nimble_bond";/' .pio/libdeps/release_PCB3/NimBLE-Arduino/src/nimble/nimble/host/store/config/src/ble_store_nvs.c
sed -i .bak 's/#define NIMBLE_NVS_NAMESPACE                     "nimble_bond"/const char* NIMBLE_NVS_NAMESPACE               = "nimble_bond";/' .pio/libdeps/release_PCB5/NimBLE-Arduino/src/nimble/nimble/host/store/config/src/ble_store_nvs.c
