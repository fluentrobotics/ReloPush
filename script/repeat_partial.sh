#!/bin/bash



#./repeat_dubins_only.sh -t var_robot_2o.txt
#./repeat_dubins_only.sh -t var_robot_3o.txt
#./repeat_dubins_only.sh -t var_robot_4o.txt
#./repeat_dubins_only.sh -t var_robot_5o.txt
#./repeat_dubins_only.sh -t var_robot_6o.txt

./repeat_mp_only.sh -t var_robot_2o.txt
./repeat_mp_only.sh -t var_robot_3o.txt
./repeat_mp_only.sh -t var_robot_4o.txt
./repeat_mp_only.sh -t var_robot_5o.txt
./repeat_mp_only.sh -t var_robot_6o.txt


./repeat.sh -t var_robot_2o.txt
./repeat.sh -t var_robot_3o.txt
./repeat.sh -t var_robot_4o.txt
./repeat.sh -t var_robot_5o.txt
./repeat.sh -t var_robot_6o.txt

