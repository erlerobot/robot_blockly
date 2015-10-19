cmake_minimum_required(VERSION 2.8.3)
project(ros_rosimple)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES pwm.msg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#SET(CMAKE_CXX_FLAGS "fpermissive")
add_definitions ( -fpermissive)

add_executable(ros_erle_pwm_writer examples/ros_erle_pwm_writer.cpp)
target_link_libraries(ros_erle_pwm_writer ${catkin_LIBRARIES})
add_dependencies(ros_erle_pwm_writer ros_erle_pwm_generate_messages_cpp)

add_executable(ros_erle_pwm_main src/ros_erle_pwm_main.cpp src/GPIO_BBB.cpp src/RCInput.cpp src/RCInput_PRU.cpp src/RCOutput_PRU.cpp src/RC_Channel.cpp src/dsm.cpp src/sbus.cpp)
target_link_libraries(ros_erle_pwm_main ${catkin_LIBRARIES})
add_dependencies(ros_erle_pwm_main ros_erle_pwm_generate_messages_cpp)

file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/examples/*)
  catkin_install_python(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	TARGETS ros_erle_pwm_main ros_erle_pwm_writer
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

