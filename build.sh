#!/bin/bash
# Build System for B1 ESC
#
# Work in progress.
#

PACKEXE="./tools/pack "

# firmware build codes
FIRM_CODE=\
"0x00010000"\ # default firmware build

mkdir rel

# build all
for b in $FIRM_CODE
do

echo "building $b..."

make clean
make boot BUILD=$b
cp *.hex rel
make clean

done


# all done!
