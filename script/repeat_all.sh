#!/bin/bash

./repeat_dubins_only.sh -t data_2o1r.txt
./repeat_dubins_only.sh -t data_3o_2.txt
./repeat_dubins_only.sh -t data_4o.txt
./repeat_dubins_only.sh -t data_5o.txt
./repeat_dubins_only.sh -t data_6o.txt


./repeat.sh -t data_2o1r.txt
./repeat.sh -t data_3o_2.txt
./repeat.sh -t data_4o.txt
./repeat.sh -t data_5o.txt
./repeat.sh -t data_6o.txt

./repeat_mp_only.sh -t data_2o1r.txt
./repeat_mp_only.sh -t data_3o_2.txt
./repeat_mp_only.sh -t data_4o.txt
./repeat_mp_only.sh -t data_5o.txt
./repeat_mp_only.sh -t data_6o.txt