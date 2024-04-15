cmake_minimum_required(VERSION 3.5)
project(ros_whill)

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
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros_whill REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(laser_geometry REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpeedPack.msg"
  "srv/SetSpeedProfile.srv"
)

add_subdirectory(./src/whill) 

add_executable(ros2_whill src/ros2_whill.cpp src/odom.cpp src/utils/rotation_tools.cpp src/utils/unit_convert.cpp)
ament_target_dependencies(ros2_whill rclcpp std_srvs sensor_msgs nav_msgs geometry_msgs tf2_ros ros_whill serial)
target_link_libraries(ros2_whill lib_whill)

add_executable(scan_fusion src/scan_fusion.cpp)
ament_target_dependencies(scan_fusion 
  rclcpp
  sensor_msgs
  laser_geometry
  sensor_msgs
  tf2_ros
  PCL
  pcl_conversions)

install(TARGETS 
  ros2_whill
  scan_fusion
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  xacro
  params
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
