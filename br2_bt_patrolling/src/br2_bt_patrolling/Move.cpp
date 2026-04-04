// Copyright 2019 Intelligent Robotics Lab
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
#include <iostream>
#include <vector>
#include <memory>

#include "br2_bt_patrolling/Move.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace br2_bt_patrolling
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: br2_bt_patrolling::BtActionNode<move_base_msgs::MoveBaseAction>(xml_tag_name, action_name,
    conf)
{
}

void
Move::on_tick()
{
  geometry_msgs::PoseStamped goal;
  getInput("goal", goal);

  goal_.target_pose = goal;
}

BT::NodeStatus
Move::on_success()
{
  ROS_INFO("navigation Succeeded");

  return BT::NodeStatus::SUCCESS;
}


}  // namespace br2_bt_patrolling

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<br2_bt_patrolling::Move>(
        name, "move_base", config);
    };

  factory.registerBuilder<br2_bt_patrolling::Move>(
    "Move", builder);
}
