cmake_minimum_required(VERSION 3.0.2)
project(br2_fsm_bumpgo_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bumpgo
  src/br2_fsm_bumpgo_cpp/BumpGoNode.cpp
  src/bumpgo_main.cpp
)
target_link_libraries(bumpgo ${catkin_LIBRARIES})

install(TARGETS bumpgo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
