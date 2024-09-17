#!/bin/bash

#./repeat.sh -t data_4o.txt
#./repeat.sh -t data_5o.txt
#./repeat.sh -t data_6o2.txt
#./repeat.sh -t data_8o.txt

./repeat.sh -t data_3o_2.txt
./repeat_dubins_only.sh -t data_8o.txt
./repeat_dubins_only.sh -t data_3o_2.txt
./repeat_dubins_only.sh -t data_4o.txt
./repeat_dubins_only.sh -t data_5o.txt
./repeat_dubins_only.sh -t data_6o2.txt

./repeat_mp_only.sh -t data_8o.txt
./repeat_mp_only.sh -t data_3o_2.txt
./repeat_mp_only.sh -t data_4o.txt
./repeat_mp_only.sh -t data_5o.txt
./repeat_mp_only.sh -t data_6o2.txt