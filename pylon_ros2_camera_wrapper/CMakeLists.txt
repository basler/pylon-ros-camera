cmake_minimum_required(VERSION 3.27)

# Suppress CMP0144 warning from third-party CMake modules (e.g. PCL/FLANN)
# that do not declare the policy themselves.
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

project(pylon_ros2_camera_wrapper)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(pylon_ros2_camera_component REQUIRED)

set(CAMERA_WRAPPER_DEPENDENCIES
	rclcpp
	rclcpp_components
	rcutils
	pylon_ros2_camera_component
)

add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME}
	${CAMERA_WRAPPER_DEPENDENCIES}
)

### installation

install(TARGETS
			${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME}
		PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)

  find_package(cv_bridge REQUIRED)
  find_package(image_geometry REQUIRED)

  if(cv_bridge_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(cv_bridge_HPP)
  endif()

  if(image_geometry_VERSION VERSION_GREATER_EQUAL 3.3.0)
    add_compile_definitions(image_geometry_HPP)
  endif()

  set(CAMERA_TEST_DEPENDENCIES
    ${CAMERA_WRAPPER_DEPENDENCIES}
    cv_bridge
    image_geometry
  )

### test

  add_library(action_client_test_grab_images SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_grab_images_action_client.cpp
  )

  target_compile_definitions(action_client_test_grab_images
    PRIVATE "TEST_GRAB_IMAGES_ACTION_CLIENT_BUILDING_DLL")

  ament_target_dependencies(action_client_test_grab_images
    ${CAMERA_TEST_DEPENDENCIES}
  )

  rclcpp_components_register_node(action_client_test_grab_images
    PLUGIN "pylon_ros2_camera::TestGrabImagesActionClient"
    EXECUTABLE test_grab_images_action_client
  )

  add_library(action_client_test_grab_blaze_data SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_grab_blaze_data_action_client.cpp
  )

  target_compile_definitions(action_client_test_grab_blaze_data
    PRIVATE "TEST_GRAB_BLAZE_DATA_ACTION_CLIENT_BUILDING_DLL")

  ament_target_dependencies(action_client_test_grab_blaze_data
    ${CAMERA_TEST_DEPENDENCIES}
  )

  rclcpp_components_register_node(action_client_test_grab_blaze_data
    PLUGIN "pylon_ros2_camera::TestGrabBlazeDataActionClient"
    EXECUTABLE test_grab_blaze_data_action_client
  )

  add_library(action_client_test_grab_image SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_grab_image_action_client.cpp
  )

  target_compile_definitions(action_client_test_grab_image
    PRIVATE "TEST_GRAB_IMAGE_ACTION_CLIENT_BUILDING_DLL")

  ament_target_dependencies(action_client_test_grab_image
    ${CAMERA_TEST_DEPENDENCIES}
  )

  rclcpp_components_register_node(action_client_test_grab_image
    PLUGIN "pylon_ros2_camera::TestGrabImageActionClient"
    EXECUTABLE test_grab_image_action_client
  )

  install(TARGETS
        action_client_test_grab_images
        action_client_test_grab_image
        action_client_test_grab_blaze_data
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
