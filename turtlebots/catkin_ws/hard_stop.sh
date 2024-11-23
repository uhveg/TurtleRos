#!/bin/bash

message="{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

rostopic pub -r 1 /bot3/cmd_vel geometry_msgs/Twist "$message" &
rostopic pub -r 1 /bot6/cmd_vel geometry_msgs/Twist "$message" &
rostopic pub -r 1 /bot7/cmd_vel geometry_msgs/Twist "$message" &
rostopic pub -r 1 /bot9/cmd_vel geometry_msgs/Twist "$message" &

read -p "Press Enter to stop publishing.."

kill %1 %2 %3 %4