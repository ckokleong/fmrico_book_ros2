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
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <ros/ros.h>
#include <ros/package.h>


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "patrolling_node");

  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_battery_checker_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_recharge_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_get_waypoint_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_track_objects_bt_node"));

  std::string pkgpath = ros::package::getPath("br2_bt_patrolling");
  std::string xml_file = pkgpath + "/behavior_tree_xml/patrolling.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  ros::Rate rate(10);

  bool finish = false;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
