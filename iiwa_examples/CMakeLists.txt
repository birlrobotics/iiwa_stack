cmake_minimum_required(VERSION 2.8.3)
project(iiwa_examples)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  cmake_modules
  roscpp
  rospy
  message_generation 
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(DIRECTORY msg FILES
  Tag_MultiModal.msg
)

add_service_files(
  FILES 
  State_Switch.srv
  )

catkin_python_setup()

generate_messages(
DEPENDENCIES
std_msgs
geometry_msgs
sensor_msgs
iiwa_msgs
)

catkin_package(
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS 
  iiwa_msgs 
  roscpp
  rospy 
  rospack
  std_msgs
  iiwa_msgs
)

#############
## Install ##
#############
install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch )
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



