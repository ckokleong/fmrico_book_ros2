cmake_minimum_required(VERSION 3.0.2)
project(br2_basics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})

add_executable(publisher_class src/publisher_class.cpp)
target_link_libraries(publisher_class ${catkin_LIBRARIES})

add_executable(subscriber_class src/subscriber_class.cpp)
target_link_libraries(subscriber_class ${catkin_LIBRARIES})

add_executable(executors src/executors.cpp)
target_link_libraries(executors ${catkin_LIBRARIES})

add_executable(logger src/logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})

add_executable(logger_class src/logger_class.cpp)
target_link_libraries(logger_class ${catkin_LIBRARIES})

add_executable(param_reader src/param_reader.cpp)
target_link_libraries(param_reader ${catkin_LIBRARIES})

install(TARGETS
  publisher
  publisher_class
  subscriber_class
  executors
  logger
  logger_class
  param_reader
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
