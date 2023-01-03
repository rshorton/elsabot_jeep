#!/usr/bin/bash

start() {
  /home/ubuntu/jeep_ws/src/elsabot_jeep/desktop_startup/run_jeep.sh
}

stop() {
  echo "stopping"
}

case $1 in
  start|stop) "$1" ;;
esac

