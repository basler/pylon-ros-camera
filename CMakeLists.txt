cmake_minimum_required(VERSION 2.8.3)
project(pylon_camera)

SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-g -Wall -Wno-unknown-pragmas -Wno-delete-non-virtual-dtor -Wno-unused-variable")

option(WITH_OPENCV "Build Pylon Camera with Rectification, Extended Auto Brightness and Sequencer (needs OpenCV)" ON)

find_package(OpenCV)
if(NOT OpenCV_FOUND)
	MESSAGE(STATUS "--- Could not find OpenCV!")
	set(WITH_OPENCV OFF)
endif()

if (WITH_OPENCV)
	MESSAGE(STATUS "+++ Building with OpenCV support")
	add_definitions(-DWITH_OPENCV)
	# OpenCV is not a catkin pkg
else ()
	MESSAGE(STATUS "--- Building WITHOUT OpenCV support")
endif ()

find_package(catkin REQUIRED COMPONENTS roscpp image_transport pylon_camera_msgs)

include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include" )

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  $ENV{PYLON_ROOT}/include
  $ENV{PYLON_ROOT}/genicam/library/CPP/include
  
  MESAGE(STATUS "$ENV{PYLON_ROOT}/include")
)
if (WITH_OPENCV)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
endif (WITH_OPENCV)



###################################
####### Architecture Check ########
###################################

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if(${ARCHITECTURE} MATCHES "arm")
	MESSAGE(STATUS "+++ Found ARM Architecture: ${ARCHITECTURE}" )
	link_directories(
		$ENV{PYLON_ROOT}/lib
		$ENV{PYLON_ROOT}/genicam/bin/Linux32_ARM
		$ENV{PYLON_ROOT}/genicam/bin/Linux32_ARM/GenApi/Generic
	)
elseif(${ARCHITECTURE} STREQUAL "x86_64")
    	MESSAGE(STATUS "+++ Found x86_64 Architecture")
	link_directories(
		$ENV{PYLON_ROOT}/lib64
		$ENV{PYLON_ROOT}/genicam/bin/Linux64_x64
		$ENV{PYLON_ROOT}/genicam/bin/Linux64_x64/GenApi/Generic
	)
else()
	MESSAGE(WARNING "+++ Unknown Architecture!")
endif()

###################################
#### Declare a cpp executable #####
###################################

if (WITH_OPENCV)
	file(GLOB_RECURSE ${PROJECT_NAME}_FILES
	    src/*.cpp
	)
        # We are too lazy to list all files, so even if we dont need (and
        # it would even fail) all files, we want to use GLOB recurse and
        # exclude just the not used files.
        list(REMOVE_ITEM ${PROJECT_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/write_magazino_id_to_camera.cpp")
        if(${OpenCV_VERSION_MAJOR} LESS 3)
        	# HDR only availbale for OpenCV 3.0
        	MESSAGE(STATUS "OpenCV Major Version is ${OpenCV_VERSION_MAJOR}. Need Major Version > 3 for HDR support. Will not compile hdr_generator.cpp")
        	list(REMOVE_ITEM ${PROJECT_NAME}_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/hdr_generator.cpp")
        endif(${OpenCV_VERSION_MAJOR} LESS 3)
            
	MESSAGE(STATUS "FILES " ${${PROJECT_NAME}_FILES})
	add_executable(pylon_camera_node src/pylon_camera_node.cpp ${${PROJECT_NAME}_FILES})
else (WITH_OPENCV)
	add_executable(pylon_camera_node 
		src/main.cpp
		src/pylon_camera_node.cpp
    	src/pylon_camera_parameter.cpp
    	src/pylon_interface.cpp
	)
endif (WITH_OPENCV)

add_executable(write_magazino_id_to_camera src/write_magazino_id_to_camera.cpp)
target_link_libraries(write_magazino_id_to_camera 
					  GenApi_gcc40_v2_3
					  GCBase_gcc40_v2_3
					  pylonbase
					  pylongigesupp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pylon_camera_node pylon_camera_generate_messages_cpp)

add_dependencies(pylon_camera_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pylon_camera_node 
					  ${catkin_LIBRARIES}
					  #${OpenCV_LIBS}
					  GenApi_gcc40_v2_3
					  GCBase_gcc40_v2_3
					  pylonbase
					  pylongigesupp
)

if(WITH_OPENCV)
	target_link_libraries(pylon_camera_node 
						  ${OpenCV_LIBS}
	)
endif(WITH_OPENCV)	

install(TARGETS pylon_camera_node write_magazino_id_to_camera RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY calib/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/calib)
FILE(GLOB py_files "scripts/*.py")
install(PROGRAMS ${py_files}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
#############
## Testing ##
#############

add_test( ${PROJECT_NAME}_smoke_test           bash -c "(sleep 4 && sleep `rand -M 4` && rosnode kill ${PROJECT_NAME} ) & rosrun ${PROJECT_NAME} pylon_camera_node  __name:=${PROJECT_NAME}")

set_tests_properties(${PROJECT_NAME}_smoke_test        PROPERTIES TIMEOUT 10)



#option(USE_SQL_DB "Use QT Sql connection" ON)
#if (USE_SQL_DB)
#    MESSAGE(STATUS "+++ WITH QT SQL")
#  add_definitions(-DWITH_QT_DB)
#  #find_package(catkin REQUIRED sqlconnection)
#endif (USE_SQL_DB)


#if (USE_SQL_DB)
# find_package(catkin REQUIRED sqlconnection actionlib)
# rosbuild_prepare_qt4(QtCore QtGui QtSql)
#else (USE_SQL_DB)
# find_package(catkin REQUIRED 
# book_gripper_msgs
# roscpp
# sensor_msgs
# std_msgs
# actionlib 
# cv_bridge
# image_transport
#)
#endif (USE_SQL_DB)





#file(GLOB_RECURSE HEADERS
#    include/pylon_camera/*.h
#    include/pylon_camera/*.hpp
#)

#file(GLOB_RECURSE ${PROJECT_NAME}_COMMON_HEADERS
#    include/pylon_camera/pylon_camera_parameter.h
#    include/pylon_camera/pylon_interface.h
#    include/pylon_camera/pylon_raw_interface.h
#)
#file(GENERATE OUTPUT ${PROJECT_NAME}_WITHCV_HEADERS
#    include/pylon_camera/image_rectifier.h
#    include/pylon_camera/pylon_hdr_interface.h
#)
#file(GLOB_RECURSE ${PROJECT_NAME}_COMMON_SRCS
#    include/pylon_camera/pylon_camera_parameter.cpp
#    include/pylon_camera/pylon_interface.cpp
#    include/pylon_camera/pylon_raw_interface.cpp
#)
#file(GLOB_RECURSE ${PROJECT_NAME}_WITHCV_SRCS
#    include/pylon_camera/image_rectifier.cpp
#    include/pylon_camera/pylon_hdr_interface.cpp
#)
#MESSAGE(STATUS "WITHCV_HEADERS " ${${PROJECT_NAME}_WITHCV_HEADERS})
#if (WITH_OPENCV)
#	file(GLOB_RECURSE ${PROJECT_NAME}_FILES
#		${${PROJECT_NAME}_COMMON_SRCS}
#		${${PROJECT_NAME}_WITHCV_SRCS}
#		# add headers so that QT shows them in project view:
#		${${PROJECT_NAME}_COMMON_HEADERS} 
#		${${PROJECT_NAME}_WITHCV_HEADERS}
#	)
#elseif (WITH_OPENCV)
#	file(GLOB_RECURSE ${PROJECT_NAME}_FILES
#		${${PROJECT_NAME}_COMMON_SRCS}
#		# add headers so that QT shows them in project view:
#		${${PROJECT_NAME}_COMMON_HEADERS} 
#	)
#endif (WITH_OPENCV) 

#MESSAGE(STATUS "FILES ${PROJECT_NAME}_FILES")

#+++++++++++++++++++++++++++++++++++
#if (PYLON_CAMERA_WITH_RECTIFICATION)
#	MESSAGE(STATUS "+++ With RECTIFICATION")
#else (PYLON_CAMERA_WITH_RECTIFICATION)
#	MESSAGE(STATUS "--- WITHOUT RECTIFICATION")
#endif (PYLON_CAMERA_WITH_RECTIFICATION)
#
#if (PYLON_CAMERA_WITH_SEQUENCER)
#	MESSAGE(STATUS "+++ With SEQUENCER")
#else (PYLON_CAMERA_WITH_SEQUENCER)
#	MESSAGE(STATUS "--- WITHOUT SEQUENCER")	
#endif (PYLON_CAMERA_WITH_SEQUENCER)
#
#if (PYLON_CAMERA_WITH_RECTIFICATION OR PYLON_CAMERA_WITH_SEQUENCER)
#	MESSAGE(STATUS "+++ Building with OpenCV support")
#	option(WITH_OPENCV "Using OpenCV" ON)
#	add_definitions(-DWITH_OPENCV)
#	# OpenCV is not a catkin pkg
#	find_package(OpenCV REQUIRED)
#	if(OpenCV_VERSION VERSION_LESS "3.0")
#		MESSAGE(STATUS "+++ OpenCV Version less then 3.0")
#	else()
#		MESSAGE(STATUS "+++ OpenCV Version greater 3.0")
#	endif()
#else (PYLON_CAMERA_WITH_RECTIFICATION OR PYLON_CAMERA_WITH_SEQUENCER)
#	MESSAGE(STATUS "--- -> Building WITHOUT OpenCV support")
#	option(WITH_OPENCV "Using OpenCV" OFF)
#endif (PYLON_CAMERA_WITH_RECTIFICATION OR PYLON_CAMERA_WITH_SEQUENCER)
