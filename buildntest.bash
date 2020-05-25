clear
cd ~/ros2_dd_ws
echo -e "\e[32mSTEP 1: Building agent code...\e[0m" 
colcon build --symlink-install
echo -e "\e[32mSTEP 2: Testing agent code...\e[0m" 
colcon test --packages-select rosrect-listener-agent
echo -e "\e[32mSTEP 3: Getting failures...\e[0m" 
colcon test-result --verbose
echo -e "\e[32mSTEP 4: Getting all test results...\e[0m" 
colcon test-result --all
cd ~/ros2_dd_ws/src/rosrect-listener-agent/