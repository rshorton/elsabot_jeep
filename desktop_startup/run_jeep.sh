#!/bin/bash

export SPEECH_SDK_PATH=/home/elsabot/ms_voice/speechsdk/
export LINOROBOT2_BASE=4wd
export LINOROBOT2_LASER_SENSOR=rplidar
export LINOROBOT2_DEPTH_SENSOR=

cd ~/robot_ws
xdg-open ros_web/robot_jeep_battery.html
source install/setup.bash && python3 src/elsabot_jeep/elsabot_jeep/ros2_web_bridge_launch.py &
source install/setup.bash && ros2 launch elsabot_jeep initial_testing.py joy:=true rviz:=false
	
