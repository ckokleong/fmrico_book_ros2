
catkin_add_gtest(bt_action_test bt_action_test.cpp)
target_link_libraries(bt_action_test ${catkin_LIBRARIES} ${behaviortree_cpp_v3_LIBRARIES})

add_executable(bt_forward bt_forward_main.cpp)
target_link_libraries(bt_forward ${catkin_LIBRARIES} ${behaviortree_cpp_v3_LIBRARIES} ${ZMQ_LIBRARIES})

install(TARGETS
  bt_forward
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
