#!/bin/bash

clear

docker build -t error_resolution_diagnoser_ros2 .

docker stop agent

docker rm agent 

docker run -it \
--env-file runtime.env \
--network=host \
--name=agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
error_resolution_diagnoser_ros2:latest  \
ros2 launch error_resolution_diagnoser_ros2 error_resolution_diagnoser_ros2_launch.py