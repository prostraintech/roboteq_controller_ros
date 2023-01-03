cmake_minimum_required(VERSION 3.5)
project(roboteq_controller)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(roboteq_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(/tmp/usr/local/include/)

set(node_plugins "")

add_executable(roboteq_controller_node src/roboteq_controller_node.cpp)
ament_target_dependencies(roboteq_controller_node rclcpp std_msgs roboteq_interfaces serial geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  roboteq_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/roboteq_proc_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()