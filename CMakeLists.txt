cmake_minimum_required(VERSION 2.8.3)
project(camera_control_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  actionlib_msgs
)

################ MSG ################
FILE(GLOB msg_files "msg/*.msg")
foreach(file ${msg_files})
    GET_FILENAME_COMPONENT(file_short ${file}  NAME)
    list( APPEND msg_base_files ${file_short})
endforeach()

add_message_files(
  FILES
  ${msg_base_files}
)
MESSAGE(STATUS "+++ Services: ${msg_base_files}")
################ MSG ################


################ ACTIONS ################
FILE(GLOB action_files "action/*.action")
foreach(file ${action_files})
    GET_FILENAME_COMPONENT(file_short ${file}  NAME)
    list( APPEND action_base_files ${file_short})
endforeach()

add_action_files(DIRECTORY action FILES ${action_base_files})
MESSAGE(STATUS "+++ Actions: ${action_base_files}")
################ ACTIONS ################

################ SRV ################
FILE(GLOB srv_files "srv/*.srv")
foreach(file ${srv_files})
    GET_FILENAME_COMPONENT(file_short ${file}  NAME)
    list( APPEND srv_base_files ${file_short})
endforeach()

MESSAGE(STATUS "+++ Services: ${srv_base_files}")
################ SRV ################


add_service_files(
  DIRECTORY srv
  FILES
  ${srv_base_files}
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs actionlib_msgs
)

