#!/bin/sh
# Lay out every C source of the application, its tests and the team's MQTT
# client (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c, .h) to Barr-C 2018.
#
#   tools/barr_format.sh
#
# Step 1, clang-format (.clang-format): 80 columns, 4-space indents, no
# tabs, braces on their own lines and around every block, spacing
# (rules 1.2, 1.3, 3.1 to 3.5).
# Step 2, tools/barr_post_format.py: what clang-format cannot do -- a final
# else on every else-if chain (8.2d), each break aligned with its case
# (8.3a) and the end-of-file comment (3.3c).
#
# Always run both steps, in this order.  Written for clang-format 23
# (pip install clang-format==23.1.5) and python3.
set -e
cd "$(dirname "$0")/.."
files="$(find app_program testing -name '*.c' -o -name '*.h' | sort)
    lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
    lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h"
clang-format -i $files
python3 tools/barr_post_format.py $files
