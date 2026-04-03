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

#include "ros/ros.h"

class LoggerNode
{
public:
  LoggerNode()
  : counter_(0)
  {
    timer_ = nh_.createTimer(
      ros::Duration(0.5), &LoggerNode::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent &)
  {
    ROS_INFO("Hello %d", counter_++);
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  int counter_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "logger_node");

  LoggerNode node;

  ros::spin();

  return 0;
}
