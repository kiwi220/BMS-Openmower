cmake_minimum_required(VERSION 3.0.2)
project(jk_bms_ble)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs diagnostic_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy sensor_msgs diagnostic_msgs)

catkin_install_python(PROGRAMS scripts/jk_bms_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
