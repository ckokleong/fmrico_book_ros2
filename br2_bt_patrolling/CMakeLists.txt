cmake_minimum_required(VERSION 3.0.2)
project(br2_bt_patrolling)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  behaviortree_cpp_v3
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  std_srvs
  roslib
)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    br2_recharge_bt_node
    br2_patrol_bt_node
    br2_move_bt_node
    br2_get_waypoint_bt_node
    br2_battery_checker_bt_node
    br2_track_objects_bt_node
  CATKIN_DEPENDS
    roscpp
    actionlib
    behaviortree_cpp_v3
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    std_srvs
    roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(br2_recharge_bt_node SHARED src/br2_bt_patrolling/Recharge.cpp)
add_library(br2_patrol_bt_node SHARED src/br2_bt_patrolling/Patrol.cpp)
add_library(br2_move_bt_node SHARED src/br2_bt_patrolling/Move.cpp)
add_library(br2_get_waypoint_bt_node SHARED src/br2_bt_patrolling/GetWaypoint.cpp)
add_library(br2_battery_checker_bt_node SHARED src/br2_bt_patrolling/BatteryChecker.cpp)
add_library(br2_track_objects_bt_node SHARED src/br2_bt_patrolling/TrackObjects.cpp)
list(APPEND plugin_libs
  br2_recharge_bt_node
  br2_patrol_bt_node
  br2_move_bt_node
  br2_get_waypoint_bt_node
  br2_battery_checker_bt_node
  br2_track_objects_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(patrolling_main src/patrolling_main.cpp)
target_link_libraries(patrolling_main ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  patrolling_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY behavior_tree_xml launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(tests)
endif()

