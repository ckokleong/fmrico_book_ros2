
catkin_add_gtest(bt_action_test bt_action_test.cpp)
target_link_libraries(bt_action_test ${catkin_LIBRARIES} br2_track_objects_bt_node)
