cmake_minimum_required(VERSION 3.8)
project(pylon_ros2_camera_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/CurrentParams.msg"
  "msg/ComponentStatus.msg"
)

set(SRV_FILES
  "srv/GetIntegerValue.srv"
  "srv/GetFloatValue.srv"
  "srv/SetBinning.srv"
  "srv/SetBrightness.srv"
  "srv/SetExposure.srv"
  "srv/SetGain.srv"
  "srv/SetGamma.srv"
  "srv/SetROI.srv"
  "srv/SetSleeping.srv"
  "srv/SetWhiteBalance.srv"
  "srv/SetIntegerValue.srv"
  "srv/SetFloatValue.srv"
  "srv/SetStringValue.srv"
)

set(ACTION_FILES
  "action/GrabImages.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs std_srvs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

