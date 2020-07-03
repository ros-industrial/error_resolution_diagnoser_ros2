#!/bin/bash

clear

docker build -t cognicept_diagnostics_agent_ros2 .

docker stop cgs_diagnostics_agent_ros2

docker rm cgs_diagnostics_agent_ros2 

docker run -it \
--env-file runtime.env \
--network=host \
--name=cgs_diagnostics_agent_ros2  \
--volume="${HOME}/.cognicept/agent/logs:/root/.cognicept/agent/logs" \
cognicept_diagnostics_agent_ros2:latest  \
ros2 launch rosrect-listener-agent-ros2 listener-agent-launch.py

# --env="ROS_MASTER_URI=http://localhost:11311" \