cmake_minimum_required(VERSION 3.27)

project(pylon_ros2_camera_test)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES Release)
  add_compile_options(-Wno-deprecated-declarations)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# pylon_ros2_camera_interfaces is a sibling workspace package that must be
# built before this package.  Give a clear diagnostic instead of the generic
# CMake "config file not found" message.
find_package(pylon_ros2_camera_interfaces QUIET)
if(NOT pylon_ros2_camera_interfaces_FOUND)
  message(FATAL_ERROR
    "\n[pylon_ros2_camera_test] Could not find pylon_ros2_camera_interfaces.\n"
    "The driver must be built before this test package.\n"
    "Use the following command to build everything in the right order:\n"
    "  colcon build --packages-up-to pylon_ros2_camera_test\n")
endif()

set(CAMERA_TEST_DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  rcutils
  pylon_ros2_camera_interfaces
  sensor_msgs
  std_srvs
)

include_directories(include)

# ── 2D camera test component ──────────────────────────────────────────────────
#
# Tests 2D camera features (image grab, binning, ROI, encoding) on top of the
# generic tests defined in CameraTestGeneric.
# Add new 2D-specific tests directly to CameraTest2D, or subclass it for a
# specific 2D camera family.

add_library(camera_test_2d_component SHARED
  src/camera_test_2d.cpp
)

target_compile_definitions(camera_test_2d_component
  PRIVATE "CAMERA_TEST_2D_BUILDING_DLL")

target_link_libraries(camera_test_2d_component PUBLIC
  ${pylon_ros2_camera_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  rclcpp_components::component_manager
  rcutils::rcutils
)

rclcpp_components_register_node(camera_test_2d_component
  PLUGIN "pylon_ros2_camera_test::CameraTest2D"
  EXECUTABLE camera_test_2d
)

# ── 3D camera test component ──────────────────────────────────────────────────
#
# Tests 3D camera features (point-cloud grab, depth range, filters) on top of
# the generic tests defined in CameraTestGeneric.
# Add new 3D-specific tests directly to CameraTest3D, or subclass it for a
# specific 3D camera family (e.g. CameraTestMyNew3D : CameraTest3D).

add_library(camera_test_3d_component SHARED
  src/camera_test_3d.cpp
)

target_compile_definitions(camera_test_3d_component
  PRIVATE "CAMERA_TEST_3D_BUILDING_DLL")

target_link_libraries(camera_test_3d_component PUBLIC
  ${pylon_ros2_camera_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  rclcpp_components::component_manager
  rcutils::rcutils
)

rclcpp_components_register_node(camera_test_3d_component
  PLUGIN "pylon_ros2_camera_test::CameraTest3D"
  EXECUTABLE camera_test_3d
)

# ── Installation ──────────────────────────────────────────────────────────────

install(TARGETS
  camera_test_2d_component
  camera_test_3d_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
)

# ── Lint ──────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
