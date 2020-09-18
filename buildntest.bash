clear
echo -e "\e[32mSTEP 1: Building agent code...\e[0m" 
colcon build --symlink-install
source /home/ros2_ws/install/setup.bash
echo -e "\e[32mSTEP 2: Testing agent code...\e[0m" 
colcon test --packages-select error_resolution_diagnoser_ros2
echo -e "\e[32mSTEP 3: Getting failures...\e[0m" 
colcon test-result --verbose
echo -e "\e[32mSTEP 4: Getting all test results...\e[0m" 
colcon test-result --all