FROM ros:eloquent-ros-core

RUN apt-get update && \
    apt-get install -y --no-install-recommends screen \
    libcpprest-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    g++ \
    make \
    ros-eloquent-launch-testing*

RUN apt-get update && \
    apt-get install -y  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install python-dateutil

WORKDIR /home/ros2_ws
COPY . src/error_resolution_diagnoser_ros2

RUN /ros_entrypoint.sh colcon build --symlink-install && sed -i '$isource "/home/ros2_ws/install/setup.bash"' /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]