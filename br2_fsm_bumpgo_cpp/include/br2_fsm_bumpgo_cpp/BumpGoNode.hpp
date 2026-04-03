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

#ifndef BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_
#define BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace br2_fsm_bumpgo_cpp
{

class BumpGoNode
{
public:
  BumpGoNode();

private:
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void control_cycle(const ros::TimerEvent&);

  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;
  static const int STOP = 3;
  int state_;
  ros::Time state_ts_;

  void go_state(int new_state);
  bool check_forward_2_back();
  bool check_forward_2_stop();
  bool check_back_2_turn();
  bool check_turn_2_forward();
  bool check_stop_2_forward();

  const ros::Duration TURNING_TIME;
  const ros::Duration BACKING_TIME;
  const ros::Duration SCAN_TIMEOUT;

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.3f;
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher vel_pub_;
  ros::Timer timer_;

  sensor_msgs::LaserScan::ConstPtr last_scan_;
};

}  // namespace br2_fsm_bumpgo_cpp

#endif  // BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_
