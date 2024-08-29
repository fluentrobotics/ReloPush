#!/bin/bash

T="default"

while getopts "t": flag;
do
    case "${flag}" in
        t) T=$OPTARG
        echo "file: "$T
        ;;
    esac
done

echo "bash sees: "$T


# Relative path to the file in the testdata directory
file="../testdata/$T"

# Check if the file exists
if [[ ! -f "$file" ]]; then
  echo "File not found!"
  exit 1
fi

echo $file
#lines=$(wc -l < "$file")



for j in $( seq 0 30 )
	do
		echo $T
		rosrun reloPush reloPush $T $j 1 mp_only
	done
