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
#include <utility>

#include "br2_bt_bumpgo/IsObstacle.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"

namespace br2_bt_bumpgo
{

IsObstacle::IsObstacle(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  scan_received_(false)
{
  laser_sub_ = nh_.subscribe(
    "/input_scan", 100, &IsObstacle::laser_callback, this);

  last_reading_time_ = ros::Time::now();
}

void
IsObstacle::laser_callback(const sensor_msgs::LaserScan::ConstPtr & msg)
{
  last_scan_ = *msg;
  scan_received_ = true;
}

BT::NodeStatus
IsObstacle::tick()
{
  if (!scan_received_) {
    return BT::NodeStatus::FAILURE;
  }

  double distance = 1.0;
  getInput("distance", distance);

  if (last_scan_.ranges[last_scan_.ranges.size() / 2] < distance) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace br2_bt_bumpgo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<br2_bt_bumpgo::IsObstacle>("IsObstacle");
}
