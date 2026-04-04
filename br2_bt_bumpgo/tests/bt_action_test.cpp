// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <list>
#include <memory>
#include <vector>
#include <set>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/ros.h"

#include "gtest/gtest.h"


class VelocitySinkNode
{
public:
  VelocitySinkNode()
  {
    vel_sub_ = nh_.subscribe(
      "/output_vel", 100, &VelocitySinkNode::vel_callback, this);
  }

  void vel_callback(const geometry_msgs::Twist::ConstPtr & msg)
  {
    vel_msgs_.push_back(*msg);
  }

  std::list<geometry_msgs::Twist> vel_msgs_;

private:
  ros::NodeHandle nh_;
  ros::Subscriber vel_sub_;
};


TEST(bt_action, turn_btn)
{
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_turn_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Turn />
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);
  bool finish = false;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 30, 1);

  geometry_msgs::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.angular.z, 0.1);
  ASSERT_NEAR(one_twist.linear.x, 0.0, 0.0000001);
}

TEST(bt_action, back_btn)
{
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_back_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Back />
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);
  bool finish = false;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 30, 1);

  geometry_msgs::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_LT(one_twist.linear.x, -0.1);
  ASSERT_NEAR(one_twist.angular.z, 0.0, 0.0000001);
}

TEST(bt_action, forward_btn)
{
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_forward_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Forward />
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);
  auto current_status = BT::NodeStatus::FAILURE;
  int counter = 0;
  while (counter++ < 30 && ros::ok()) {
    current_status = tree.rootNode()->executeTick();
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_EQ(current_status, BT::NodeStatus::RUNNING);
  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 30, 1);

  geometry_msgs::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.linear.x, 0.1);
  ASSERT_NEAR(one_twist.angular.z, 0.0, 0.0000001);
}

TEST(bt_action, is_obstacle_btn)
{
  ros::NodeHandle nh;
  auto scan_pub = nh.advertise<sensor_msgs::LaserScan>("input_scan", 1);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_is_obstacle_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <IsObstacle/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);

  sensor_msgs::LaserScan scan;
  scan.ranges.push_back(2.0);
  for (int i = 0; i < 10; i++) {
    scan_pub.publish(scan);
    ros::spinOnce();
    rate.sleep();
  }

  BT::NodeStatus current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::FAILURE);

  scan.ranges[0] = 0.3;
  for (int i = 0; i < 10; i++) {
    scan_pub.publish(scan);
    ros::spinOnce();
    rate.sleep();
  }

  current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::SUCCESS);

  xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <IsObstacle distance="0.5"/>
      </BehaviorTree>
    </root>)";
  tree = factory.createTreeFromText(xml_bt, blackboard);

  scan.ranges[0] = 0.3;
  for (int i = 0; i < 10; i++) {
    scan_pub.publish(scan);
    ros::spinOnce();
    rate.sleep();
  }

  current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::SUCCESS);

  scan.ranges[0] = 0.6;
  for (int i = 0; i < 10; i++) {
    scan_pub.publish(scan);
    ros::spinOnce();
    rate.sleep();
  }

  current_status = tree.rootNode()->executeTick();
  ASSERT_EQ(current_status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bt_action_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
