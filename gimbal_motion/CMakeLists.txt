cmake_minimum_required(VERSION 3.8)
project(gimbal_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(gimbal_motion_node src/gimbal_motion_node.cpp)
ament_target_dependencies(gimbal_motion_node rclcpp geometry_msgs dynamixel_sdk)

install(TARGETS
  gimbal_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()