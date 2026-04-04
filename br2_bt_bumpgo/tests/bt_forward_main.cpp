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

#include "ros/ros.h"


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "forward_node");
  ros::NodeHandle nh;

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
  bool finish = false;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
