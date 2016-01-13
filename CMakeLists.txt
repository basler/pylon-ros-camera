cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

add_definitions("-std=gnu++11")
#set(CMAKE_CXX_FLAGS "-g -Wall -Wno-unknown-pragmas -Wno-delete-non-virtual-dtor -Wno-unused-variable")
set(
    CATKIN_COMPONENTS
     actionlib
     camera_control_msgs
     image_transport
     roscpp
     sensor_msgs
)

find_package(Pylon QUIET REQUIRED)
find_package(
    catkin REQUIRED
    COMPONENTS ${CATKIN_COMPONENTS} roslint rostest
)

catkin_package(
    INCLUDE_DIRS 
     include
    LIBRARIES 
     ${PROJECT_NAME}
    CATKIN_DEPENDS 
     ${CATKIN_COMPONENTS}
)

set(
    ROSLINT_CPP_OPTS 
     "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

roslint_cpp(
    src/${PROJECT_NAME}/exposure_search_parameter.cpp
    src/${PROJECT_NAME}/main.cpp
    src/${PROJECT_NAME}/pylon_camera_node.cpp
    src/${PROJECT_NAME}/pylon_camera_parameter.cpp
    src/${PROJECT_NAME}/pylon_camera.cpp
    src/${PROJECT_NAME}/write_magazino_id_to_camera.cpp
    include/${PROJECT_NAME}/exposure_search_parameter.h
    include/${PROJECT_NAME}/pylon_camera_node.h
    include/${PROJECT_NAME}/pylon_camera_parameter.h
    include/${PROJECT_NAME}/pylon_camera.h
    include/${PROJECT_NAME}/internal/pylon_camera.h
    include/${PROJECT_NAME}/internal/impl/pylon_camera_base.hpp
    include/${PROJECT_NAME}/internal/impl/pylon_camera_dart.hpp
    include/${PROJECT_NAME}/internal/impl/pylon_camera_gige.hpp
    include/${PROJECT_NAME}/internal/impl/pylon_camera_usb.hpp
)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

include_directories(SYSTEM ${Pylon_INCLUDE_DIRS} )
include_directories( ${catkin_INCLUDE_DIRS} )

# Add library
add_library(
    ${PROJECT_NAME}
     src/${PROJECT_NAME}/exposure_search_parameter.cpp
     src/${PROJECT_NAME}/pylon_camera.cpp
     src/${PROJECT_NAME}/pylon_camera_node.cpp
     src/${PROJECT_NAME}/pylon_camera_parameter.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${Pylon_LIBRARIES}
)

add_dependencies(
    ${PROJECT_NAME}
     ${catkin_EXPORTED_TARGETS}
)

# Add pylon_camera_node
add_executable(
    ${PROJECT_NAME}_node
     src/${PROJECT_NAME}/main.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

add_executable(
    write_magazino_id_to_camera
     src/${PROJECT_NAME}/write_magazino_id_to_camera.cpp
)

target_link_libraries(
    write_magazino_id_to_camera
     ${Pylon_LIBRARIES}
)

add_dependencies(
    ${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
)

catkin_python_setup()

install(
    DIRECTORY 
     launch/
    DESTINATION 
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
    DIRECTORY 
     config/
    DESTINATION 
     ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

file(GLOB ${PROJECT_NAME}_SCRIPTS 
        "scripts/*.py" 
)

install(
	PROGRAMS
	 scripts/brightness_evaluation.py
	 scripts/brightness_publisher.py
	 scripts/compatibility_exposure_action.py
	 scripts/exp_caller.py
	 scripts/file_sequencer.py
	 scripts/result_bag_to_action.py
	 scripts/sequence_to_file.py
	 test/test_scripts/single_acquisition_test.py
	DESTINATION 
	 ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    TARGETS 
     ${PROJECT_NAME} 
     ${PROJECT_NAME}_node 
     write_magazino_id_to_camera
    LIBRARY DESTINATION 
     ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION 
     ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY 
     include/${PROJECT_NAME}/
    DESTINATION 
     ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN 
     "*.h"
    PATTERN "internal" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  #add_rostest(test/open_various_camera_types.test)
  
  # smoke_test
  add_test(${PROJECT_NAME}_smoke_test
           /bin/bash "-c" "(sleep 4 && sleep `rand -M 16` && rosnode kill ${PROJECT_NAME} ) & rosrun ${PROJECT_NAME} pylon_camera_node  __name:=${PROJECT_NAME}")
  set_tests_properties(${PROJECT_NAME}_smoke_test PROPERTIES TIMEOUT 20)
  
  # open_different_camera_types_tests
#  add_rostest(test/open_gige.test)
#  add_rostest(test/open_usb.test)
  
  # test for acquisition of one image_raw and one_cam_info
  add_rostest(test/single_acquisition_dart.test)
  add_rostest(test/single_acquisition_gige.test)
  add_rostest(test/single_acquisition_usb.test)
   
  # frame_rate_tests
  add_rostest(test/frame_rate_dart.test)
  add_rostest(test/frame_rate_gige.test)
  add_rostest(test/frame_rate_usb.test)
endif()


###############
## QtCreator ##
###############
# entry for QtCreator to show all files
file(GLOB children ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
  if(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    list(APPEND ${PROJECT_NAME}_extra_files ${dir_files})
  endif()
endforeach()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${${PROJECT_NAME}_extra_files})
