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

#include "br2_fsm_bumpgo_cpp/BumpGoNode.hpp"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace br2_fsm_bumpgo_cpp
{

using std::placeholders::_1;

BumpGoNode::BumpGoNode()
: state_(FORWARD),
  TURNING_TIME(2.0),
  BACKING_TIME(2.0),
  SCAN_TIMEOUT(1.0)
{
  scan_sub_ = nh_.subscribe("input_scan", 1,
    &BumpGoNode::scan_callback, this);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("output_vel", 10);
  timer_ = nh_.createTimer(ros::Duration(0.05),
    &BumpGoNode::control_cycle, this);

  state_ts_ = ros::Time::now();
}

void
BumpGoNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  last_scan_ = msg;
}

void
BumpGoNode::control_cycle(const ros::TimerEvent&)
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_stop()) {
        go_state(STOP);
      }

      if (check_forward_2_back()) {
        go_state(BACK);
      }
      break;
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        go_state(TURN);
      }
      break;
    case TURN:
      out_vel.angular.z = SPEED_ANGULAR;

      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }

      break;
    case STOP:
      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_.publish(out_vel);
}

void
BumpGoNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = ros::Time::now();
}

bool
BumpGoNode::check_forward_2_back()
{
  // going forward when deteting an obstacle
  // at 0.5 meters with the front laser read
  size_t pos = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
}

bool
BumpGoNode::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = ros::Time::now() - last_scan_->header.stamp;
  return elapsed > SCAN_TIMEOUT;
}

bool
BumpGoNode::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = ros::Time::now() - last_scan_->header.stamp;
  return elapsed < SCAN_TIMEOUT;
}

bool
BumpGoNode::check_back_2_turn()
{
  // Going back for 2 seconds
  return (ros::Time::now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNode::check_turn_2_forward()
{
  // Turning for 2 seconds
  return (ros::Time::now() - state_ts_) > TURNING_TIME;
}

}  // namespace br2_fsm_bumpgo_cpp
