// Copyright 2024 Intelligent Robotics Lab
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

#include <fstream>
#include <thread>

#include "yaets/tracing.hpp"

#include "ros/ros.h"
#include "std_msgs/Int32.h"

using std::placeholders::_1;


yaets::TraceSession session("session1.log");

class ProducerNode
{
public:
  explicit ProducerNode(ros::NodeHandle & nh)
  {
    pub_1_ = nh.advertise<std_msgs::Int32>("topic_1", 100);
    pub_2_ = nh.advertise<std_msgs::Int32>("topic_2", 100);
    timer_ = nh.createWallTimer(
      ros::WallDuration(0.001), &ProducerNode::timer_callback, this);
  }

  void timer_callback(const ros::WallTimerEvent &)
  {
    message_.data += 1;
    pub_1_.publish(message_);
    message_.data += 1;
    pub_2_.publish(message_);
  }

private:
  ros::Publisher pub_1_, pub_2_;
  ros::WallTimer timer_;
  std_msgs::Int32 message_;
};

class ConsumerNode
{
public:
  explicit ConsumerNode(ros::NodeHandle & nh)
  {
    // Use a separate callback queue for subscriptions to emulate
    // a MutuallyExclusive callback group (subscriptions won't run
    // concurrently with each other, but can run in parallel with the timer)
    custom_nh_.setCallbackQueue(&custom_queue_);

    sub_2_ = custom_nh_.subscribe("topic_2", 100, &ConsumerNode::cb_2, this);
    sub_1_ = custom_nh_.subscribe("topic_1", 100, &ConsumerNode::cb_1, this);

    timer_ = nh.createWallTimer(
      ros::WallDuration(0.01), &ConsumerNode::timer_cb, this);
  }

  void cb_1(const std_msgs::Int32::ConstPtr & msg)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.0005));
  }

  void cb_2(const std_msgs::Int32::ConstPtr & msg)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.0005));
  }

  void timer_cb(const ros::WallTimerEvent &)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.005));
  }

  void waste_time(const ros::WallDuration & duration)
  {
    ros::WallTime start = ros::WallTime::now();
    while (ros::WallTime::now() - start < duration) {}
  }

  ros::CallbackQueue & getCustomQueue()
  {
    return custom_queue_;
  }

private:
  ros::Subscriber sub_1_;
  ros::Subscriber sub_2_;
  ros::WallTimer timer_;
  ros::NodeHandle custom_nh_;
  ros::CallbackQueue custom_queue_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "executors_cbg_node");
  ros::NodeHandle nh;

  ProducerNode producer(nh);
  ConsumerNode consumer(nh);

  // Spin the custom callback queue on a single thread
  // (emulates MutuallyExclusive callback group behavior)
  std::thread custom_thread([&consumer]() {
      while (ros::ok()) {
        consumer.getCustomQueue().callAvailable(ros::WallDuration(0.001));
      }
    });

  // Multi-threaded spinning for the main queue
  // (equivalent to ROS2 MultiThreadedExecutor with 8 threads)
  ros::AsyncSpinner spinner(8);
  spinner.start();
  ros::waitForShutdown();

  custom_thread.join();

  return 0;
}
