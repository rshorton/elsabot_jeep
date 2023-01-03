Files used to support bringing-up the robot by double-clicking a desktop icon.

- Elsabot Jeep Bringup - desktop launcher file that should be copied to the
  Ubuntu desktop. This file runs the run_jeep.sh script.
- run_jeep.sh - bash script to run the robot launch scripts

Files used to support starting robot with systemd:
- robot.service 
   systemd service file.  Create symlink to this file:
       cd /etc/systemd/system
       sudo ln -s /home/<replace with user>/<you ros ws>/src/elsabot_jeep/desktop_startup/robot.service robot.service
   Edit robot.service to specify the 'user' account to be used when running the launch scripts.
- startup.sh
   startup script specified by the robot.service which runs the 'run_jeep.sh' script.
- run_jeep.sh - Same as above