#!/bin/bash
# Patches the installed NimBLE-Arduino library in each PlatformIO environment.
# Must be re-run after the NimBLE library is (re)installed, and before building.
#
#  - Raises CONFIG_BT_NIMBLE_MAX_BONDS above MAX_ADDRESSES so a new device can
#    always bond before the oldest is evicted (see src/xInput.cpp / src/keyboard.cpp).
#  - Raises CONFIG_BT_NIMBLE_MAX_CCCDS.
#  - Makes NIMBLE_NVS_NAMESPACE an externally-defined string.
#
# Idempotent: safe to run multiple times.

set -u
cd "$(dirname "$0")"

ENVS="devel release_PCB3 release_PCB5"
MAX_BONDS=5
MAX_CCCDS=20

patched=0
for env in $ENVS; do
    dir=".pio/libdeps/$env/NimBLE-Arduino"
    cfg="$dir/src/nimconfig.h"
    nvs="$dir/src/nimble/nimble/host/store/config/src/ble_store_nvs.c"

    if [ ! -f "$cfg" ]; then
        echo "skip: $env (not installed)"
        continue
    fi

    # Patch only the active defines (the line immediately following #undef),
    # leaving the #ifndef fallbacks further down untouched.
    sed -i .bak -E \
        "/^#undef CONFIG_BT_NIMBLE_MAX_BONDS\$/{n;s/^#define CONFIG_BT_NIMBLE_MAX_BONDS[[:space:]].*/#define CONFIG_BT_NIMBLE_MAX_BONDS $MAX_BONDS/;}" \
        "$cfg"
    sed -i .bak -E \
        "/^#undef CONFIG_BT_NIMBLE_MAX_CCCDS\$/{n;s/^#define CONFIG_BT_NIMBLE_MAX_CCCDS[[:space:]].*/#define CONFIG_BT_NIMBLE_MAX_CCCDS $MAX_CCCDS/;}" \
        "$cfg"

    if [ -f "$nvs" ]; then
        sed -i .bak 's/#define NIMBLE_NVS_NAMESPACE                     "nimble_bond"/const char* NIMBLE_NVS_NAMESPACE               = "nimble_bond";/' "$nvs"
    fi

    if grep -qE "^#define CONFIG_BT_NIMBLE_MAX_BONDS $MAX_BONDS$" "$cfg"; then
        echo "patched: $env (MAX_BONDS=$MAX_BONDS, MAX_CCCDS=$MAX_CCCDS)"
        patched=$((patched+1))
    else
        echo "ERROR: failed to patch $cfg" >&2
    fi
done

if [ "$patched" -eq 0 ]; then
    echo "ERROR: no NimBLE environments were patched." >&2
    exit 1
fi

echo "NimBLE patched in $patched environment(s)."
