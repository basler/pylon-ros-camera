cmake_minimum_required(VERSION 2.8.3)
project(camera_control_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  sensor_msgs
  std_msgs
)

################ MSG ################
file(GLOB msg_files "msg/*.msg")
foreach(file ${msg_files})
    GET_FILENAME_COMPONENT(file_short ${file}  NAME)
    list( APPEND msg_base_files ${file_short})
endforeach()

add_message_files(
  FILES
  ${msg_base_files}
)
message(STATUS "+++ Services: ${msg_base_files}")
################ MSG ################


################ ACTIONS ################
file(GLOB action_files "action/*.action")
foreach(file ${action_files})
    GET_FILENAME_COMPONENT(file_short ${file}  NAME)
    list( APPEND action_base_files ${file_short})
endforeach()

add_action_files(DIRECTORY action FILES ${action_base_files})
message(STATUS "+++ Actions: ${action_base_files}")
################ ACTIONS ################

################ SRV ################
file(GLOB srv_files "srv/*.srv")
foreach(file ${srv_files})
    GET_FILENAME_COMPONENT(file_short ${file}  NAME)
    list( APPEND srv_base_files ${file_short})
endforeach()

message(STATUS "+++ Services: ${srv_base_files}")
################ SRV ################


add_service_files(
  DIRECTORY srv
  FILES
  ${srv_base_files}
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS 
  actionlib_msgs
  message_runtime 
  sensor_msgs 
  std_msgs 
)

