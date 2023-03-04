# Elsabot Jeep Robot

Top-level bring up scripts for the Jeep base version of the Elsabot robot.  

Some parts of these scripts and configuration files are from the Linorobot2 project:  https://github.com/linorobot/linorobot2

The firmware for the base microcontroller is based on the linorobot2_hardware project.  See this fork for the firmware used for the Elsabot Jeep base (jeep branch): https://github.com/rshorton/linorobot2_hardware/tree/jeep


## Installation Summary

**Hardware**: See the above jeep link for a full list

**Computer**: Raspberry Pi4 (8G)

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
    + ros2 launch elsabot_jeep bringup.launch.py joy:=true rviz:=false
    + Use telop with game controller to control jeep
2. Start navigation using terminal shell 2:
    + ros2 launch elsabot_jeep navigation.launch.py rviz:=false
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
