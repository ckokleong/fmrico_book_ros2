cmake_minimum_required(VERSION 3.0.2)
project(br2_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch params maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

