cmake_minimum_required(VERSION 3.5)
project(rosrect-listener-agent)

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
set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)
find_package(OpenSSL REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(rosrect-listener-agent-lib src/listener_agent.cpp src/backend_api.cpp src/robot_event.cpp src/state_manager.cpp)
target_link_libraries(
  rosrect-listener-agent-lib
  cpprestsdk::cpprest
  ${Boost_LIBRARIES}
)

ament_target_dependencies(rosrect-listener-agent-lib rclcpp rcl_interfaces ament_index_cpp)

add_executable(rosrect-listener-agent src/listener_agent.cpp src/backend_api.cpp src/robot_event.cpp src/state_manager.cpp)
target_link_libraries(
  rosrect-listener-agent
  # rosrect-listener-agent-lib
  cpprestsdk::cpprest
  ${Boost_LIBRARIES}
)

ament_target_dependencies(rosrect-listener-agent rclcpp rcl_interfaces ament_index_cpp)

install(TARGETS
   rosrect-listener-agent
   DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   rosrect-listener-agent-lib
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
  
  ament_target_dependencies(backend_test_node rosrect-listener-agent-lib rclcpp rcl_interfaces ament_index_cpp)
  ament_target_dependencies(robotevent_test_node rosrect-listener-agent-lib rclcpp rcl_interfaces ament_index_cpp)
  ament_target_dependencies(statemanager_test_node rosrect-listener-agent-lib rclcpp rcl_interfaces ament_index_cpp)
  
  target_link_libraries(backend_test_node rosrect-listener-agent-lib cpprestsdk::cpprest ${Boost_LIBRARIES})
  target_link_libraries(robotevent_test_node rosrect-listener-agent-lib cpprestsdk::cpprest ${Boost_LIBRARIES})
  target_link_libraries(statemanager_test_node rosrect-listener-agent-lib cpprestsdk::cpprest ${Boost_LIBRARIES})
endif()

ament_package()