cmake_minimum_required(VERSION 3.0.2)
project(br2_fsm_bumpgo_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/bump_go_main
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
