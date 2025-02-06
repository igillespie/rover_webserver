#!/bin/bash
source /home/pi/myenv/bin/activate
source /home/pi/ros2_jazzy/install/setup.bash
source /home/pi/camera_ws/install/setup.bash
source /home/pi/ros2_extras/install/setup.bash
python3 /home/pi/rover_webserver/webserver/server.py