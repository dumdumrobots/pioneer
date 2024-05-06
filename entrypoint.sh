#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /home/ros/pioneer_ws/install/setup.bash

sudo chmod 777 /dev/input/event*
sudo chmod 777 /dev/ttyUBS0

echo "Arguments: $@"

exec $@