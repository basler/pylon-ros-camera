cmake_minimum_required(VERSION 3.27)

# Suppress CMP0144 warning from third-party CMake modules (e.g. PCL/FLANN)
# that do not declare the policy themselves.
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

project(pylon_ros2_camera_component)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

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

# Check for blaze camera support (requires pylon Supplementary Package for blaze)
find_file(PYLON_BLAZE_HEADER "pylon/BlazeInstantCamera.h" PATHS ${Pylon_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(PYLON_BLAZE_HEADER)
    message(STATUS "Pylon blaze support: ENABLED")
    add_compile_definitions(HAVE_PYLON_BLAZE)
else()
    message(STATUS "Pylon blaze support: DISABLED (pylon/BlazeInstantCamera.h not found)")
endif()

# Check for Stereo mini camera support (requires pylon Supplementary Package for Stereo mini)
find_file(PYLON_STEREO_MINI_HEADER "pylon/StereoMiniInstantCamera.h" PATHS ${Pylon_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(PYLON_STEREO_MINI_HEADER)
    message(STATUS "Pylon Stereo mini support: ENABLED")
    add_compile_definitions(HAVE_PYLON_STEREO_MINI)
else()
    message(STATUS "Pylon Stereo mini support: DISABLED (pylon/StereoMiniInstantCamera.h not found)")
endif()

# Check for Stereo ace camera support (requires pylon Supplementary Package for Stereo ace)
find_file(PYLON_STEREO_ACE_HEADER "pylon/StereoAceInstantCamera.h" PATHS ${Pylon_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(PYLON_STEREO_ACE_HEADER)
    message(STATUS "Pylon Stereo ace support: ENABLED")
    add_compile_definitions(HAVE_PYLON_STEREO_ACE)
else()
    message(STATUS "Pylon Stereo ace support: DISABLED (pylon/StereoAceInstantCamera.h not found)")
endif()

find_package(ament_cmake REQUIRED)
find_package(pylon_ros2_camera_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(diagnostic_updater REQUIRED)

if(cv_bridge_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(cv_bridge_HPP)
endif()

if(image_geometry_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(image_geometry_HPP)
endif()

set(PYLON_ROS2_CAMERA_DEPENDENCIES
	pylon_ros2_camera_interfaces
	rclcpp
	rclcpp_action
	rclcpp_components
	rcutils
	sensor_msgs
	cv_bridge
	pcl_ros
	camera_info_manager
	image_geometry
	image_transport
	diagnostic_updater
)

set(PYLON_ROS2_CAMERA_FILE_PREFIX pylon_ros2_camera)

set(PYLON_ROS2_CAMERA_LIBRARIES
	${Pylon_LIBRARIES}
)

# create ament index resource referencing the libraries in the binary folder

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
    	$<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM
	PUBLIC
		$<BUILD_INTERFACE:${Pylon_INCLUDE_DIRS}>
)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE "PYLON_ROS2_CAMERA_BUILDING_DLL"
)

target_link_libraries(${PROJECT_NAME}
	PRIVATE
		${PYLON_ROS2_CAMERA_LIBRARIES}
	PUBLIC
		${pylon_ros2_camera_interfaces_TARGETS}
		${sensor_msgs_TARGETS}
		camera_info_manager::camera_info_manager
		cv_bridge::cv_bridge
		diagnostic_updater::diagnostic_updater
		image_geometry::image_geometry
		image_transport::image_transport
		pcl_ros::pcl_ros_tf
		rclcpp::rclcpp
		rclcpp_action::rclcpp_action
		rclcpp_components::component
		rclcpp_components::component_manager
		rcutils::rcutils
		sensor_msgs::sensor_msgs_library
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pylon_ros2_camera::PylonROS2CameraNode"
  EXECUTABLE pylon_ros2_camera_node
)

### tools

# IP Auto Config
add_executable(ip_auto_config
	${CMAKE_CURRENT_SOURCE_DIR}/src/tools/ip_auto_config.cpp
)

target_include_directories(ip_auto_config SYSTEM
	PRIVATE
		$<BUILD_INTERFACE:${Pylon_INCLUDE_DIRS}>
)

target_link_libraries(ip_auto_config
	PRIVATE ${PYLON_ROS2_CAMERA_LIBRARIES}
)

# set user id
add_executable(set_device_user_id
	${CMAKE_CURRENT_SOURCE_DIR}/src/tools/set_device_user_id.cpp
)

target_include_directories(set_device_user_id SYSTEM
	PRIVATE
		$<BUILD_INTERFACE:${Pylon_INCLUDE_DIRS}>
)

target_link_libraries(set_device_user_id
	PRIVATE ${PYLON_ROS2_CAMERA_LIBRARIES}
)

### installation

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ip_auto_config
    set_device_user_id
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${PYLON_ROS2_CAMERA_DEPENDENCIES})

ament_package()
