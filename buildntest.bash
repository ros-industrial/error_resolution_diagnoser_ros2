clear
cd ~/ros2_dd_ws
colcon build --symlink-install
colcon test --packages-select rosrect-listener-agent
colcon test-result --verbose
colcon test-result --all
cd ~/ros2_dd_ws/src/rosrect-listener-agent/