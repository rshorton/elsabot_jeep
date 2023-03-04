# Elsabot Jeep Robot

Top-level bring up scripts for the Jeep base version of the Elsabot robot.  

Some parts of these scripts and configuration files are from the Linorobot2 project:  https://github.com/linorobot/linorobot2

The firmware for the base microcontroller is based on the linorobot2_hardware project.  See this fork for the firmware used for the Elsabot Jeep base (jeep branch): https://github.com/rshorton/linorobot2_hardware/tree/jeep


## Installation Summary

Hardware: See the above jeep link for a full list

Computer: Raspberry Pi4 (8G)

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

udev rules, /etc/udev/rules.d/99-usb-serial.rules 

* SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0",ATTRS{idProduct}=="0483", SYMLINK+="teensy"
* SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"

Development Machine
* Installed full ROS Humble install
* Used RVIZ on this machine.

teleop Controller
* Luna game controller in bluetooth mode paired to RPi4.
