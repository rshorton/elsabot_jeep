# Elsabot Jeep Robot

Top-level bring up scripts for the Jeep base version of the Elsabot robot.  

Some parts of these scripts and configuration files are from the Linorobot2 project:  https://github.com/linorobot/linorobot2

The firmware for the base microcontroller is based on the linorobot2_hardware project.  See this fork for the firmware used for the Elsabot Jeep base (jeep branch): https://github.com/rshorton/linorobot2_hardware/tree/jeep


## Installation Summary

**Hardware**: See the above jeep link for a full list

**Computer**: Raspberry Pi4 (8GB)

### Packages needed for this install

ROS Humble
* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    
* The desktop install was used, but a minimal install should work also.

Nav2
* sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup 
* Also needed teb_local_planner for support of Ackerman steering.  This was built from source:

  * ros2-master branch, https://github.com/rst-tu-dortmund/teb_local_planner.git 
  * ros2 branch, https://github.com/rst-tu-dortmund/costmap_converter.git
  * Also needed: sudo apt install ros-humble-libg2o

micro-ROS and agent
* Used the 'Building' steps from https://github.com/micro-ROS/micro_ros_setup/blob/humble/README.md

Other
* sudo apt install ros-humble-rplidar-ros
* sudo apt install ros-humble-laser-filters
* sudo apt install ros-humble-joint-state-publisher
* sudo apt install ros-humble-rosbridge-suite
* sudo apt install ros-humble-robot-localization
* sudo apt install ros-humble-joy-linux

Development Machine
* Installed full ROS Humble install
* Used RVIZ on this machine.

## Other Setup

udev rules, /etc/udev/rules.d/99-usb-serial.rules 

* SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0",ATTRS{idProduct}=="0483", SYMLINK+="teensy"
* SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"

teleop Controller
* Luna game controller in bluetooth mode paired to RPi4.

## Running

1. Start base functionality using terminal shell 1:
    + ros2 launch elsabot_jeep bringup.launch.py
    + Use telop with game controller to control jeep
2. Start navigation using terminal shell 2:
    + ros2 launch elsabot_jeep navigation.launch.py
3. Start RVIZ on development host  
    + Use RVIZ to set initial position and then issue nav commands.
  
Map creation for navigation
  1. Use this step instead of 2 above:
     + ros2 launch elsabot_jeep slam.launch.py rviz:=false
  2. Move robot around using telop to build map.
  3. Save map using:
     + ros2 run nav2_map_server map_saver_cli -f <path_to/elsabot_jeep/maps/your_map_name> --ros-args -p save_map_timeout:=10000.
  4. Edit launch/navigation.launch.py to use your map.

  ## Other

  See the **Special Teleop Control** section of the Jeep hardware Readme.

  If you installed the full desktop install of Ubuntu on the RPi4, then it is best to disable the graphics subsystem on the RPi4 after setup to avoid unnecessarily wasting memory and CPU.  Use these command to disable/renable it and then reboot.

  * Disable
    + sudo systemctl set-default multi-user.target

  * Enable
    + sudo systemctl set-default graphical.target

## Gazebo Simulation

Steps for running Gazebo and navigation looping test.  Also requires elsabot_bt and
robot_pose_publisher packages.

```
export IGN_GAZEBO_RESOURCE_PATH=/home/scott/robot_ws/src
ros2 launch elsabot_jeep gazebo.launch.py
ros2 launch elsabot_jeep navigation.launch.py use_sim_time:=true rviz:=true use_keep_out:=false
<Use RViz to set the 2D pose estimate.>
ros2 launch elsabot_bt elsabot_bt_only.launch.py bt_xml:=/home/scott/robot_ws/src/elsabot_bt/bt_xml/bt_nav_loop.xml
```

Tested with Gazebo Fortress.

Behavior you should see:

* Simple Gazebo world that mirrors the backyard map well enough for localization to work when using a real backyard map for navigation. 
* The Behavior tree will start the jeep navigating a path around the yard.  It will repeat the number of repeat loops specified in the behavior tree.
* After looping the specified number of times the behavior tree will exit.

Real-world demo (the hardware mounted on the back of the jeep not used for this demo, only the RPi4): 
https://www.youtube.com/watch?v=f-G0z6sHZ58
