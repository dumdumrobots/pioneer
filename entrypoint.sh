#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /home/ros/pioneer_ws/install/setup.bash

echo "Arguments: $@"

exec $@