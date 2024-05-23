#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /home/ros/pioneer_ws/install/setup.bash

sudo chmod 777 /dev/input/event*
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 -R /docker_shared

echo "Arguments: $@"

exec $@