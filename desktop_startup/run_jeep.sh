#!/bin/bash

show_help ()
{
    echo "run_jeep -b"
}

show_webui=false

while getopts ":b" option; do
    case $option in
        b)
            show_webui=true
            ;;

        h)
            show_help
            exit 0
            ;;

        \?)
            echo "Error: Invalid option"
            exit 1        
            ;;
   esac
done

export SPEECH_SDK_PATH=/home/elsabot/ms_voice/speechsdk/
# FIX
export LINOROBOT2_BASE=4wd
export LINOROBOT2_LASER_SENSOR=rplidar
export LINOROBOT2_DEPTH_SENSOR=

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd ${SCRIPT_DIR}/../../..
pwd

if [ $show_webui = true ]; then
    xdg-open ros_web/robot_jeep_battery.html
fi

logger "Starting elsabot jeep"
source install/setup.bash && ros2 launch elsabot_jeep initial_testing.py joy:=true rviz:=false &

