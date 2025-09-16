cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # action
  # "action/HeadOrientation.action"

  # msg
  "msg/CircleMic.msg"
  "msg/HeadPose.msg"
  "msg/MotorCmd.msg"
  "msg/HeadCmd.msg"

  # srv
  # "srv/SetString.srv"

  DEPENDENCIES
  std_msgs
)

ament_package()
