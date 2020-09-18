cmake_minimum_required(VERSION 3.5)
project(error_resolution_diagnoser_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)
find_package(OpenSSL REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(error_resolution_diagnoser_ros2_lib src/listener_agent.cpp src/backend_api.cpp src/robot_event.cpp src/state_manager.cpp)
target_link_libraries(
  error_resolution_diagnoser_ros2_lib
  cpprestsdk::cpprest
  ${Boost_LIBRARIES}
)

ament_target_dependencies(error_resolution_diagnoser_ros2_lib rclcpp rcl_interfaces ament_index_cpp nav_msgs geometry_msgs)

add_executable(error_resolution_diagnoser_ros2 src/listener_agent.cpp src/backend_api.cpp src/robot_event.cpp src/state_manager.cpp)
target_link_libraries(
  error_resolution_diagnoser_ros2
  cpprestsdk::cpprest
  ${Boost_LIBRARIES}
)

ament_target_dependencies(error_resolution_diagnoser_ros2 rclcpp rcl_interfaces ament_index_cpp nav_msgs geometry_msgs)

install(TARGETS
   error_resolution_diagnoser_ros2
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   error_resolution_diagnoser_ros2_lib
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  # include_directories(test)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake)
  
  ament_add_gtest(backend_test_node test/utests_backendapi.cpp)
  ament_add_gtest(robotevent_test_node test/utests_robotevent.cpp)
  ament_add_gtest(statemanager_test_node test/utests_statemanager.cpp)
  add_launch_test(test/listener_integration_launch_ros_test.py)
  #add_launch_test(test/listener_integration_launch_db_test.py)
  add_launch_test(test/listener_integration_launch_telemetry_test.py)
  
  ament_target_dependencies(backend_test_node rclcpp rcl_interfaces ament_index_cpp nav_msgs geometry_msgs)
  ament_target_dependencies(robotevent_test_node rclcpp rcl_interfaces ament_index_cpp nav_msgs geometry_msgs)
  ament_target_dependencies(statemanager_test_node rclcpp rcl_interfaces ament_index_cpp nav_msgs geometry_msgs)
  
  target_link_libraries(backend_test_node error_resolution_diagnoser_ros2_lib cpprestsdk::cpprest ${Boost_LIBRARIES})
  target_link_libraries(robotevent_test_node error_resolution_diagnoser_ros2_lib cpprestsdk::cpprest ${Boost_LIBRARIES})
  target_link_libraries(statemanager_test_node error_resolution_diagnoser_ros2_lib cpprestsdk::cpprest ${Boost_LIBRARIES})
endif()

ament_package()
