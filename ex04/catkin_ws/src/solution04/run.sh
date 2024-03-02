#!/bin/bash

echo
echo "=========================== convert mat to bag ============================="
echo
roslaunch solution04 mat2bag.launch
echo 
echo "=========================== play bag and run estimator "===========================
echo
roslaunch solution04 run.launch