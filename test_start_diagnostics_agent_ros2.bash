#!/bin/bash

clear

docker build -t rosrect_agent_ros2 .

docker stop agent

docker rm agent 

docker run -it \
--env-file runtime.env \
--network=host \
--name=agent  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
rosrect_agent_ros2:latest  \
ros2 launch rosrect-listener-agent-ros2 listener-agent-launch.py

# --env="ROS_MASTER_URI=http://localhost:11311" \