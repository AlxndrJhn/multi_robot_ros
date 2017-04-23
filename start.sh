#!/bin/bash
source /opt/ros/indigo/setup.bash
rostopic pub -1 /sync_cmd std_msgs/String "start"
