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
#include "std_msgs/Int32.h"

class PublisherNode
{
public:
  PublisherNode()
  {
    publisher_ = nh_.advertise<std_msgs::Int32>("int_topic", 10);
    timer_ = nh_.createTimer(
      ros::Duration(0.5), &PublisherNode::timer_callback, this);
  }

  void timer_callback(const ros::TimerEvent &)
  {
    message_.data += 1;
    publisher_.publish(message_);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  std_msgs::Int32 message_;
};

class SubscriberNode
{
public:
  SubscriberNode()
  {
    subscriber_ = nh_.subscribe("int_topic", 10,
      &SubscriberNode::callback, this);
  }

  void callback(const std_msgs::Int32::ConstPtr & msg)
  {
    ROS_INFO("Hello %d", msg->data);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "executors_node");

  PublisherNode pub_node;
  SubscriberNode sub_node;

  ros::spin();

  return 0;
}
