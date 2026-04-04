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
#include <iostream>

#include "br2_bt_bumpgo/Back.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace br2_bt_bumpgo
{

Back::Back(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/output_vel", 100);
}

void
Back::halt()
{
}

BT::NodeStatus
Back::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = ros::Time::now();
  }

  geometry_msgs::Twist vel_msgs;
  vel_msgs.linear.x = -0.3;
  vel_pub_.publish(vel_msgs);

  auto elapsed = ros::Time::now() - start_time_;

  if (elapsed < ros::Duration(3.0)) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_bumpgo::Back>("Back");
}
