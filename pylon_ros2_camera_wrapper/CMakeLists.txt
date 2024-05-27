cmake_minimum_required(VERSION 3.8)
project(pylon_ros2_camera_wrapper)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(pylon_ros2_camera_component REQUIRED)
find_package(cv_bridge REQUIRED)

set(CAMERA_WRAPPER_DEPENDENCIES
	rclcpp
	rclcpp_components
	rcutils
	pylon_ros2_camera_component
	cv_bridge
)

add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME}
	${CAMERA_WRAPPER_DEPENDENCIES}
)

### test

add_library(action_client_test_grab_images SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/test/test_grab_images_action_client.cpp
)

target_compile_definitions(action_client_test_grab_images
  PRIVATE "TEST_GRAB_IMAGES_ACTION_CLIENT_BUILDING_DLL")

ament_target_dependencies(action_client_test_grab_images
  ${CAMERA_WRAPPER_DEPENDENCIES}
)

rclcpp_components_register_node(action_client_test_grab_images
	PLUGIN "pylon_ros2_camera::TestGrabImagesActionClient"
	EXECUTABLE test_grab_image_action_client
)

add_library(action_client_test_grab_blaze_data SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/test/test_grab_blaze_data_action_client.cpp
)

target_compile_definitions(action_client_test_grab_blaze_data
  PRIVATE "TEST_GRAB_BLAZE_DATA_ACTION_CLIENT_BUILDING_DLL")

ament_target_dependencies(action_client_test_grab_blaze_data
  ${CAMERA_WRAPPER_DEPENDENCIES}
)

rclcpp_components_register_node(action_client_test_grab_blaze_data
	PLUGIN "pylon_ros2_camera::TestGrabBlazeDataActionClient"
	EXECUTABLE test_grab_blaze_data_action_client
)

### installation

install(TARGETS
			${PROJECT_NAME}
			action_client_test_grab_images
			action_client_test_grab_blaze_data
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME}
)

ament_package()
