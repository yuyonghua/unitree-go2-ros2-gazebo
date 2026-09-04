#!/bin/bash

# 1. 先 source ROS 2 环境（让它先把默认的环境变量建立好）
source /opt/ros/humble/setup.bash
source ~/git/yyh/go2_ws/install/setup.bash

# 2. 再配置你自己的环境变量（追加并覆盖它，保证你的路径不丢失）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/git/yyh/go2_ws/src/cyclonedds.xml

# 注意这里使用冒号追加，这样能同时保留系统默认路径和你自己的本地路径
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/git/yyh/go2_ws/src/go2_gazebo_bringup/worlds:$HOME/git/yyh/go2_ws/src/go2_gazebo_bringup/models
export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH

export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib

