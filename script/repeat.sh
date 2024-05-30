#!/bin/bash

while getopts t: flag
do
    case "${flag}" in
        t) T=${OPTARG};;
    esac
done

echo $T


# Relative path to the file in the testdata directory
file="../testdata/$T"

# Check if the file exists
if [[ ! -f "$file" ]]; then
  echo "File not found!"
  exit 1
fi

echo $file
lines=$(wc -l < "$file")



for j in $( seq 0 $lines )
	do
		#echo $$
		rosrun reloPush reloPush $T $j
	done
