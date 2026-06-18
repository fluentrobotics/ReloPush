#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <filename> <use_opt (0 or 1)>"
    exit 1
fi

# Get arguments
FILENAME="$1"
OPT="$2" # 1 for optimization, 0 for non-optimization

# Validate FLAG to ensure it's either 0 or 1
if [[ "$OPT" != "0" && "$OPT" != "1" ]]; then
    echo "Error: OPT must be 0 or 1"
    exit 1
fi

# Loop from 0 to 99
for i in {0..99}
do
    #../build/Release/ReloPush "$FILENAME" "$i" "$OPT"
    build/ReloPush-BOSS "$FILENAME" "$i" "f"
done
