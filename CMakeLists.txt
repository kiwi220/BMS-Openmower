cmake_minimum_required(VERSION 3.0.2)
project(bms_ble)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs diagnostic_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy sensor_msgs diagnostic_msgs)

catkin_install_python(PROGRAMS scripts/bms_ble_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# The bridge runs with its own Python >= 3.12 interpreter (aiobmsble), so it is
# installed as a plain file and not rewritten by catkin_install_python.
install(PROGRAMS bridge/bms_bridge.py bridge/setup_venv.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bridge)
install(FILES bridge/requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bridge)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
