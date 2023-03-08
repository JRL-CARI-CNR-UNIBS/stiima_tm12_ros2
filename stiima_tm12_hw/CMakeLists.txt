cmake_minimum_required(VERSION 3.8)
project(stiima_tm12_hw)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_definitions(LOG_JOINTS)
  message("${PROJECT_NAME}: Debug build, logging all info in share directory.")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tm_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(tm_robot_driver_plugin
SHARED
src/hardware_interface.cpp
src/tm_ros2_sct.cpp
src/tm_ros2_svr.cpp
src/tm_driver.cpp
src/tm_command.cpp
src/tm_sct_communication.cpp
src/tm_svr_communication.cpp
src/tm_communication.cpp
src/tm_robot_state.cpp
src/tm_packet.cpp
src/tm_print.cpp
src/tm_driver_utilities.cpp
src/tm_listen_node_connect.cpp
src/tm_ethernet_slave_connect.cpp
)

ament_target_dependencies(tm_robot_driver_plugin
rclcpp
ament_index_cpp
rclcpp_lifecycle
controller_manager
controller_manager_msgs
geometry_msgs
hardware_interface
tf2_geometry_msgs
pluginlib
std_msgs
sensor_msgs
tm_msgs
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(TARGETS
  tm_robot_driver_plugin
  DESTINATION lib
)

ament_package()
