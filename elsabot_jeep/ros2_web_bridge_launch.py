
import os
import sys
import subprocess

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    print("Running ros2 web bridge")
    # Fix install of rosbridge.js
    #p = subprocess.run(['node', '/home/elsabot/robot_ws/ros2-web-bridge/bin/rosbridge.js'])
    p = subprocess.run(['python3', '/opt/ros/galactic/lib/rosbridge_server/rosbridge_websocket.py'])

    rclpy.spin(depthai_publisher)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
