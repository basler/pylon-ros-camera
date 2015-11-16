cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

add_definitions("-std=gnu++11")
#set(CMAKE_CXX_FLAGS "-g -Wall -Wno-unknown-pragmas -Wno-delete-non-virtual-dtor -Wno-unused-variable")
set(CATKIN_COMPONENTS
    actionlib
    image_transport
    camera_control_msgs
    roscpp
    sensor_msgs)

find_package(PkgConfig)
pkg_check_modules(PYLON4 pylon4)
if(NOT PYLON4_FOUND)
    message(FATAL_ERROR "Could not find Pylon4 pkg-config files. Maybe you are not using the most recent version of Pylon4 from rosdep. Please run 'rosdep update' and 'rosdep install --from-path pylon_camera --ignore-src --rosdistro=indigo -y' inside your catkin_ws and try again.")
endif()

find_package(
    catkin REQUIRED
    COMPONENTS ${CATKIN_COMPONENTS}
)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

link_directories(
    ${PYLON4_LIBRARY_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_COMPONENTS}
    #DEPENDS Pylon4
)


include_directories(SYSTEM ${PYLON4_INCLUDE_DIRS} )
include_directories( ${catkin_INCLUDE_DIRS} )

# Add library
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/exposure_search_parameter.cpp
    src/${PROJECT_NAME}/pylon_camera.cpp
    src/${PROJECT_NAME}/pylon_camera_node.cpp
    src/${PROJECT_NAME}/pylon_camera_parameter.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PYLON4_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${pylon_camera_msgs_EXPORTED_TARGETS}
)

# Add node
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

# Add other executable
add_executable(write_magazino_id_to_camera
    src/${PROJECT_NAME}/write_magazino_id_to_camera.cpp
)

target_link_libraries(write_magazino_id_to_camera
    ${PYLON4_LIBRARIES}
)

catkin_python_setup()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

file(GLOB py_files "scripts/*")
install(PROGRAMS ${py_files}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node write_magazino_id_to_camera
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN "internal" EXCLUDE
)

#############
## Testing ##
#############

add_test(${PROJECT_NAME}_smoke_test           bash -c "(sleep 4 && sleep `rand -M 4` && rosnode kill ${PROJECT_NAME} ) & rosrun ${PROJECT_NAME} pylon_camera_node  __name:=${PROJECT_NAME}")

set_tests_properties(${PROJECT_NAME}_smoke_test        PROPERTIES TIMEOUT 10)




file(GLOB children ${CMAKE_SOURCE_DIR}/*)
foreach(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
endforeach()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
