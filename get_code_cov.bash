cd ~/ros2_dd_ws
# . install/setup.sh
# Testing the robot 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_CXX_OUTPUT_EXTENSION_REPLACE=ON
# Testing for code coverage
colcon test --packages-select error_resolution_diagnoser_ros2
# LCOV info file generated
# lcov --remove 'test/*' 'include/*' --capture --directory build/ --output-file ros2.info
# GCOV report 
gcovr -r . --exclude-directories test --html-details -o coverage/coverage.html
#Coverage report
# cd ..
# cp ros2_dd_ws/coverage.xml coverage.xml
# unninstalling the ros packages
#rm -r  ~/cognicept-jenkins/workspace/Cognicept\ Agents/cognicept-ros2-agent-pipeline/ros2_dd_ws