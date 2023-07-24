#!/bin/bash

cd ~/ros2_ws/
. install/local_setup.bash
source /opt/ros/humble/setup.bash
colcon build
. install/local_setup.bash
source /opt/ros/humble/setup.bash

gnome-terminal -- ros2 launch mobile_robotics_hschoon mobile_robotics_hschoon.launch.py use_sim_time:=True

gnome-terminal -- ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

gnome-terminal -- ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True 

gnome-terminal -- bash -c 'cd ~/ros2_ws/;
. install/local_setup.bash;
ros2 launch explore_lite explore.launch.py; exec bash'

gnome-terminal -- bash -c 'cd ~/ros2_ws/;
. install/local_setup.bash;
ros2 run twist_mux twist_mux --ros-args --params-file ./src/mobile_robotics_hschoon/config/twist_mux.yaml -r cmd_vel_out:=simple_diff_drive_controller/cmd_vel_unstamped; exec bash'
