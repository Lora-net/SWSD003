#!/bin/bash

# This script prepare the code base for LLCC68 driver
# It must be run from within folder <TOP_DIR>/llcc68_autogen
# To use it, the archive name must be provided in first argument.
# The archive will be created with tar bzip2 (j flag)

# Fail on error and undefined variable
set -eu

usage() { echo "Usage: $0 <LLCC68_DRIVER_ARCHIVE_NAME_TO_CREATE>" 1>&2; exit 1; }

# Check user input
if [ "$#" -ne 1 ]; then
        echo "Invalid call"
        usage
        exit
fi

ARCHIVE_NAME_BASE="$1"

# Thanks to https://stackoverflow.com/a/246128
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Variable for archive production
PROJECT_NAME="llcc68_driver"
PROJECT_VERSION=$(git describe --tags)
ARCHIVE_NAME=$ARCHIVE_NAME_BASE

# These are the folders of the project
TOP_PROJECT_FOLDER="$SCRIPT_DIR/.."
SRC_PROJECT_FOLDER="$TOP_PROJECT_FOLDER/src"
TESTS_PROJECT_FOLDER="$TOP_PROJECT_FOLDER/tests"

# These are the temporary folders created to generate the LLCC68 driver code base
TEMPORARY_FOLDER_BASE=$(mktemp -d)
SUBFOLDER_RELEASE_NAME=$PROJECT_NAME-$PROJECT_VERSION
TOP_RELEASE_FOLDER="$TEMPORARY_FOLDER_BASE/$SUBFOLDER_RELEASE_NAME"
SRC_RELEASE_FOLDER="$TOP_RELEASE_FOLDER"/src/
TESTS_RELEASE_FOLDER="$TOP_RELEASE_FOLDER"/tests/

#trap "{ rm -rf \"$TOP_RELEASE_FOLDER\"; }" EXIT

echo "========================================="
echo "Top project folder is $TOP_PROJECT_FOLDER"
echo "Temporary folder is $TOP_RELEASE_FOLDER"
echo "Archive to generate is $ARCHIVE_NAME"
echo "========================================="
echo ""

mkdir -p "$SRC_RELEASE_FOLDER"
mkdir -p "$TESTS_RELEASE_FOLDER"

cp $TOP_PROJECT_FOLDER/llcc68_autogen/.gitlab-ci.yml "$TOP_RELEASE_FOLDER"
cp $TOP_PROJECT_FOLDER/LICENSE.txt "$TOP_RELEASE_FOLDER"

cat $TOP_PROJECT_FOLDER/README.md | grep -iv "fhss" | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > "$TOP_RELEASE_FOLDER"/README.md
cat $TOP_PROJECT_FOLDER/CHANGELOG.md | grep -iv "fhss" | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > "$TOP_RELEASE_FOLDER"/CHANGELOG.md

cat $SRC_PROJECT_FOLDER/sx126x.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' -e 's/DS_SX1261-2_V1.2/DS_LLCC68_V1.0/g' > $SRC_RELEASE_FOLDER/llcc68.c
cat $SRC_PROJECT_FOLDER/sx126x.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' -e 's/SX1262/LLCC68/g' -e 's/DS_SX1261-2_V1.2/DS_LLCC68_V1.0/g' > $SRC_RELEASE_FOLDER/llcc68.h
cat $SRC_PROJECT_FOLDER/sx126x_regs.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' -e 's/DS_SX1261-2_V1.2/DS_LLCC68_V1.0/g' > $SRC_RELEASE_FOLDER/llcc68_regs.h
cat $SRC_PROJECT_FOLDER/sx126x_hal.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $SRC_RELEASE_FOLDER/llcc68_hal.h

cat $TESTS_PROJECT_FOLDER/project.yml | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/project.yml
cat $TESTS_PROJECT_FOLDER/run_tests.sh | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/run_tests.sh
cat $TESTS_PROJECT_FOLDER/sx126x_toa.h | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/llcc68_toa.h
cat $TESTS_PROJECT_FOLDER/test_sx126x_communication_status.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_communication_status.c
cat $TESTS_PROJECT_FOLDER/test_sx126x_dio_irq_control.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_dio_irq_control.c
cat $TESTS_PROJECT_FOLDER/test_sx126x_extra.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_extra.c
cat $TESTS_PROJECT_FOLDER/test_sx126x_miscellaneous.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_miscellaneous.c
cat $TESTS_PROJECT_FOLDER/test_sx126x_modulation_packet.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_modulation_packet.c
cat $TESTS_PROJECT_FOLDER/test_sx126x_operational_modes.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_operational_modes.c
cat $TESTS_PROJECT_FOLDER/test_sx126x_reg_buf_access.c | sed -e 's/sx126x/llcc68/g' -e 's/SX126X/LLCC68/g' -e 's/SX126x/LLCC68/g' > $TESTS_RELEASE_FOLDER/test_llcc68_reg_buf_access.c
cp $TOP_PROJECT_FOLDER/llcc68_autogen/test_llcc68_dio_irq_control_llcc68_only.c $TESTS_RELEASE_FOLDER

sed -i '/LLCC68_LORA_SF12/d' "$TOP_RELEASE_FOLDER"/src/llcc68.h


# Update llcc68.c

input=""$SRC_RELEASE_FOLDER"/llcc68.c"

line_start=$(awk '/llcc68_set_lora_mod_params/{ print NR; exit }' $input)
line_end=line_start

while IFS= read -r line; do
  ((line_end++))
  [[ $line = }* ]] && break
done < <(tail -n "+$line_start" $input)

sed -i "${line_start},${line_end}d" $input

((line_start-=2))
sed -i "${line_start}r ./llcc68_set_lora_mod_params.c" $input


# Update llcc68.h

input=""$SRC_RELEASE_FOLDER"/llcc68.h"

line_start=$(awk '/llcc68_irq_masks_e/{ print NR; exit }' $input)
line_end=line_start

while IFS= read -r line; do
  ((line_end++))
  [[ $line = }* ]] && break
done < <(tail -n "+$line_start" $input)

((line_end-=1))
sed -i "${line_start},${line_end}d" $input

((line_start-=1))
sed -i "${line_start}r ./llcc68_irq_mask_t.c" $input

sed -i '/LLCC68_PKT_TYPE_LR_FHSS/d' $input


# Update test_llcc68_modulation_packet.c

input="$TESTS_RELEASE_FOLDER/test_llcc68_modulation_packet.c"

line_start=$(awk '/test_llcc68_set_lora_mod_params/{ print NR; exit }' $input)
line_end=line_start

while IFS= read -r line; do
  ((line_end++))
  [[ $line = }* ]] && break
done < <(tail -n "+$line_start" $input)

((line_end-=1))
sed -i "${line_start},${line_end}d" $input

((line_start-=1))
sed -i "${line_start}r ./llcc68_set_lora_mod_params_test.c" $input


# Check if there is a mention of sx126 somewhere

if grep -rni $TOP_RELEASE_FOLDER/* -q -e 'sx126' -e 'fhss'; then
  echo "ERROR: There are still mentions to sx126x and/or fhss."
  exit 1
fi

# Generate the driver archive
tar cjf $ARCHIVE_NAME -C $TEMPORARY_FOLDER_BASE $SUBFOLDER_RELEASE_NAME
