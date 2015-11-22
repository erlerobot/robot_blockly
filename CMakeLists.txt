cmake_minimum_required(VERSION 2.8.3)
project(rosimple)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

# add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv BadTwoInts.srv)

#generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs)

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
catkin_install_python(PROGRAMS
  scripts/rosimple_backend.py
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
