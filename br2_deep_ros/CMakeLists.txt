cmake_minimum_required(VERSION 3.5)
project(br2_deep_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  vision_msgs
)

find_package(yaets REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(executors src/executors.cpp)
target_link_libraries(executors ${catkin_LIBRARIES} yaets::yaets)

add_executable(executors_cbg src/executors_cbg.cpp)
target_link_libraries(executors_cbg ${catkin_LIBRARIES} yaets::yaets)

add_executable(executors_cbg_reentrant src/executors_cbg_reentrant.cpp)
target_link_libraries(executors_cbg_reentrant ${catkin_LIBRARIES} yaets::yaets)

add_executable(wakeup src/wakeup.cpp)
target_link_libraries(wakeup yaets::yaets)

add_executable(wakeup_ros src/wakeup_ros.cpp)
target_link_libraries(wakeup_ros ${catkin_LIBRARIES} yaets::yaets)

add_executable(strategy_1 src/strategy_1.cpp)
target_link_libraries(strategy_1 ${catkin_LIBRARIES} yaets::yaets)

add_executable(strategy_2 src/strategy_2.cpp)
target_link_libraries(strategy_2 ${catkin_LIBRARIES} yaets::yaets)

add_executable(strategy_3 src/strategy_3.cpp)
target_link_libraries(strategy_3 ${catkin_LIBRARIES} yaets::yaets)

install(TARGETS
  executors
  executors_cbg
  executors_cbg_reentrant
  wakeup
  wakeup_ros
  strategy_1
  strategy_2
  strategy_3
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
