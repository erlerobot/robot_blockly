cmake_minimum_required(VERSION 2.8.3)
project(robot_blockly)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

# add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(FILES SetCurrentBlockId.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/robot_blockly_backend.py
  scripts/pywebserver.py
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

install(DIRECTORY frontend
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
