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
#include <stdexcept>
#include <cstring>
#include <thread>

#include "yaets/tracing.hpp"

#include "ros/ros.h"
#include "std_msgs/Int32.h"

using std::placeholders::_1;


yaets::TraceSession session("strategy_1.log");


void waste_time(const ros::WallDuration & duration)
{
  ros::WallTime start = ros::WallTime::now();
  while (ros::WallTime::now() - start < duration) {}
}

class ProducerNode
{
public:
  explicit ProducerNode(ros::NodeHandle & nh)
  {
    pub_ = nh.advertise<std_msgs::Int32>("int_topic", 100);
    timer_ = nh.createWallTimer(
      ros::WallDuration(0.01), &ProducerNode::timer_callback, this);
  }

  void timer_callback(const ros::WallTimerEvent &)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.0002));

    message_.data += 1;
    pub_.publish(message_);
  }

private:
  ros::Publisher pub_;
  ros::WallTimer timer_;
  std_msgs::Int32 message_;
};

class ConsumerNode
{
public:
  explicit ConsumerNode(ros::NodeHandle & nh)
  {
    sub_ = nh.subscribe("int_topic", 100, &ConsumerNode::cb, this);

    timer_ = nh.createWallTimer(
      ros::WallDuration(0.01), &ConsumerNode::timer_cb, this);
  }

  void cb(const std_msgs::Int32::ConstPtr & msg)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.0005));
  }


  void timer_cb(const ros::WallTimerEvent &)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.002));
  }

private:
  ros::Subscriber sub_;
  ros::WallTimer timer_;
};

class LoggerNode
{
public:
  explicit LoggerNode(ros::NodeHandle & nh)
  {
    sub_ = nh.subscribe("int_topic", 100, &LoggerNode::cb, this);

    timer_ = nh.createWallTimer(
      ros::WallDuration(0.01), &LoggerNode::timer_cb, this);
  }

  void cb(const std_msgs::Int32::ConstPtr & msg)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.0005));
  }


  void timer_cb(const ros::WallTimerEvent &)
  {
    TRACE_EVENT(session);

    waste_time(ros::WallDuration(0.002));
  }

private:
  ros::Subscriber sub_;
  ros::WallTimer timer_;
};


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "strategy_1_node");

  // Non-RT callback queue (default/main queue)
  ros::NodeHandle no_rt_nh;

  // RT callback queue on a separate queue
  ros::CallbackQueue rt_queue;
  ros::NodeHandle rt_nh;
  rt_nh.setCallbackQueue(&rt_queue);

  ProducerNode producer(no_rt_nh);
  ConsumerNode consumer(rt_nh);
  LoggerNode logger(no_rt_nh);

  auto rt_thread = std::thread(
    [&]() {
      sched_param sch;
      sch.sched_priority = 90;

      if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
        throw std::runtime_error{std::string("failed to set scheduler: ") + std::strerror(errno)};
      }

      while (ros::ok()) {
        rt_queue.callAvailable(ros::WallDuration(0.001));
      }
  });

  ros::spin();

  rt_thread.join();

  return 0;
}
