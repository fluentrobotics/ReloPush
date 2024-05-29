#!/bin/bash

for j in $( seq 0 20 )
	do
		#echo $$
		rosrun reloPush reloPush data_2o1r.txt $j
	done
