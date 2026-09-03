#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/git/yyh/go2_ws/cyclonedds.xml
export GZ_SIM_RESOURCE_PATH=$HOME/git/yyh/go2_ws/src/go2_gazebo_bringup/worlds:$HOME/git/yyh/go2_ws/src/go2_gazebo_bringup/models
export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/git/yyh/go2_ws/install/setup.bash
