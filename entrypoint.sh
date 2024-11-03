#!/bin/bash

# Source the ROS and workspace setup
source /opt/ros/humble/setup.sh
source vrx_ws/install/setup.bash

# Start a tmux session
tmux new-session -s mysession -d

# Attach to the tmux session
tmux attach-session -t mysession

sudo service udev status
sudo service udev start
sleep 62
sudo udevadm control --reload-rules
sudo udevadm trigger
cd ..
git clone https://github.com/naoki-mizuno/ds4_driver.git -b humble-devel
cd ds4_driver
colcon build
. install/setup.bash