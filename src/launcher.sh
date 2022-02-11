#!/bin/bash
echo "Rosbag play"
rosparam set use_sim_time true
cd
cd Downloads
rosbag play agrosavia_50_2021-12-03-19-43-03.bag --clock
