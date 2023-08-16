cmake_minimum_required(VERSION 3.8)
project(pylon_ros2_camera_component)

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

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release)
   #message(" * Release Mode")
   add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
   #message(" * Release with Debug Info Mode")
   add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
   message(" * Debug Mode")
endif()

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND} OR "$ENV{Pylon_INCLUDE_DIRS}" STREQUAL "")
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
# message("->    Pylon include directories: ${Pylon_INCLUDE_DIRS}")
# message("->    Pylon library directories: ${Pylon_LIBRARY_DIRS}")
# message("->    Pylon libraries: ${Pylon_LIBRARIES}")

find_package(ament_cmake REQUIRED)
find_package(pylon_ros2_camera_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(diagnostic_updater REQUIRED)

set(PYLON_ROS2_CAMERA_DEPENDENCIES
	pylon_ros2_camera_interfaces
	rclcpp
	rclcpp_action
	rclcpp_components
	rcutils
	sensor_msgs
	cv_bridge
	camera_info_manager
	image_geometry
	image_transport
	diagnostic_updater
)

set(PYLON_ROS2_CAMERA_FILE_PREFIX pylon_ros2_camera)

set(PYLON_ROS2_CAMERA_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${Pylon_INCLUDE_DIRS}
)

set(PYLON_ROS2_CAMERA_LIBRARY_DIRS
	${Pylon_LIBRARY_DIRS}
)

set(PYLON_ROS2_CAMERA_LIBRARIES
	${Pylon_LIBRARIES}
)

# create ament index resource referencing the libraries in the binary folder
set(node_plugins "")

add_library(${PROJECT_NAME} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/src/binary_exposure_search.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/encoding_conversions.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/${PYLON_ROS2_CAMERA_FILE_PREFIX}.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/${PYLON_ROS2_CAMERA_FILE_PREFIX}_node.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/${PYLON_ROS2_CAMERA_FILE_PREFIX}_parameter.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
    	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${Pylon_INCLUDE_DIRS}>
    	$<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE "PYLON_ROS2_CAMERA_BUILDING_DLL"
)

target_link_libraries(${PROJECT_NAME}
	${PYLON_ROS2_CAMERA_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
	${PYLON_ROS2_CAMERA_DEPENDENCIES}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "pylon_ros2_camera_component::PylonROS2CameraNode")
set(node_plugins "${node_plugins}pylon_ros2_camera_component::PylonROS2CameraNode;$<TARGET_FILE:${PROJECT_NAME}>\n")

### tools

# IP Auto Config
add_executable(ip_auto_config
	${CMAKE_CURRENT_SOURCE_DIR}/src/tools/ip_auto_config.cpp
)

target_include_directories(ip_auto_config
	PUBLIC
		$<BUILD_INTERFACE:${Pylon_INCLUDE_DIRS}>
    	$<INSTALL_INTERFACE:include>
)

target_link_libraries(ip_auto_config
	${PYLON_ROS2_CAMERA_LIBRARIES}
)

# set user id
add_executable(set_device_user_id
	${CMAKE_CURRENT_SOURCE_DIR}/src/tools/set_device_user_id.cpp
)

target_include_directories(set_device_user_id
	PUBLIC
		$<BUILD_INTERFACE:${Pylon_INCLUDE_DIRS}>
    	$<INSTALL_INTERFACE:include>
)

target_link_libraries(set_device_user_id
	${PYLON_ROS2_CAMERA_LIBRARIES}
)

### installation

install(
  TARGETS
  	${PROJECT_NAME}
	ip_auto_config
	set_device_user_id
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING
  	PATTERN "*.hpp"
  	PATTERN "internal" EXCLUDE
)

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PYLON_ROS2_CAMERA_DEPENDENCIES})

ament_package()
