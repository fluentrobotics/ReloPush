#!/bin/bash

./repeat_dubins_only.sh -t data_5o.txt
./repeat_dubins_only.sh -t data_6o.txt

./repeat_mp_only.sh -t data_5o.txt
./repeat_mp_only.sh -t data_6o.txt

./repeat.sh -t data_5o.txt
./repeat.sh -t data_6o.txt

