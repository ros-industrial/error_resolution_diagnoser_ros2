FROM ros:eloquent-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends screen \
    libcpprest-dev

RUN apt-get update && \
    apt-get install -y  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros2_ws
COPY . src/rosrect-listener-agent-ros2

RUN /ros_entrypoint.sh colcon build --symlink-install && sed -i '$isource "/home/ros2_ws/install/setup.bash"' /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]