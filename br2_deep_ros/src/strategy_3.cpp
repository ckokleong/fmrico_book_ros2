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
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection3D.h"

using std::placeholders::_1;


yaets::TraceSession session("strategy_3.log");


void waste_time(const ros::WallDuration & duration)
{
  ros::WallTime start = ros::WallTime::now();
  while (ros::WallTime::now() - start < duration) {}
}


class SensorDriverNode
{
public:
  SensorDriverNode(ros::NodeHandle & nh, ros::NodeHandle & rt_nh)
  {
    pub_ = nh.advertise<sensor_msgs::Image>("image", 100);
    // RT timer on the RT callback queue
    timer_scan_ = rt_nh.createWallTimer(
      ros::WallDuration(0.01), &SensorDriverNode::produce_data, this);
    // Non-RT timer on the main callback queue
    timer_state_ = nh.createWallTimer(
      ros::WallDuration(0.1), &SensorDriverNode::print_state, this);
  }

  void produce_data(const ros::WallTimerEvent &)
  {
    SHARED_TRACE_START("brake_process");

    waste_time(ros::WallDuration(0.0002));

    sensor_msgs::Image image_msg;
    pub_.publish(image_msg);
  }

  void print_state(const ros::WallTimerEvent &)
  {
    waste_time(ros::WallDuration(0.001));
  }

private:
  ros::Publisher pub_;
  ros::WallTimer timer_scan_, timer_state_;
};


class ObstacleDetectorNode
{
public:
  ObstacleDetectorNode(ros::NodeHandle & nh, ros::NodeHandle & rt_nh)
  {
    // RT subscription on the RT callback queue
    sub_ = rt_nh.subscribe("image", 100, &ObstacleDetectorNode::detect_obstacle, this);
    pub_ = nh.advertise<vision_msgs::Detection3D>("obstacles", 100);
    // Non-RT timer on the main callback queue
    timer_state_ = nh.createWallTimer(
      ros::WallDuration(0.1), &ObstacleDetectorNode::print_state, this);
  }

  void detect_obstacle(const sensor_msgs::Image::ConstPtr & msg)
  {
    waste_time(ros::WallDuration(0.005));

    vision_msgs::Detection3D detection_msg;
    pub_.publish(detection_msg);
  }


  void print_state(const ros::WallTimerEvent &)
  {
    waste_time(ros::WallDuration(0.001));
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::WallTimer timer_state_;
};


class LoggerNode
{
public:
  explicit LoggerNode(ros::NodeHandle & nh)
  {
    sub_ = nh.subscribe("image", 100, &LoggerNode::cb, this);

    timer_state_ = nh.createWallTimer(
      ros::WallDuration(0.01), &LoggerNode::print_state, this);
  }

  void cb(const sensor_msgs::Image::ConstPtr & msg)
  {
    waste_time(ros::WallDuration(0.0005));
  }


  void print_state(const ros::WallTimerEvent &)
  {
    waste_time(ros::WallDuration(0.002));
  }

private:
  ros::Subscriber sub_;
  ros::WallTimer timer_state_;
};


class BrakeActuatorNode
{
public:
  BrakeActuatorNode(ros::NodeHandle & nh, ros::NodeHandle & rt_nh)
  {
    // RT subscription on the RT callback queue
    sub_ = rt_nh.subscribe("obstacles", 100, &BrakeActuatorNode::react_obstacle, this);
    // Non-RT timer on the main callback queue
    timer_state_ = nh.createWallTimer(
      ros::WallDuration(0.1), &BrakeActuatorNode::print_state, this);
  }

  void react_obstacle(const vision_msgs::Detection3D::ConstPtr & msg)
  {
    waste_time(ros::WallDuration(0.002));
    SHARED_TRACE_END("brake_process");
  }


  void print_state(const ros::WallTimerEvent &)
  {
    waste_time(ros::WallDuration(0.001));
  }

private:
  ros::Subscriber sub_;
  ros::WallTimer timer_state_;
};


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "strategy_3_node");

  SHARED_TRACE_INIT(session, "brake_process");

  ros::NodeHandle nh;

  // RT callback queue (emulates the RT MultiThreadedExecutor with 3 threads)
  ros::CallbackQueue rt_queue;
  ros::NodeHandle rt_nh;
  rt_nh.setCallbackQueue(&rt_queue);

  SensorDriverNode sensor_driver(nh, rt_nh);
  ObstacleDetectorNode obstacle_detector(nh, rt_nh);
  LoggerNode logger(nh);
  BrakeActuatorNode brake_actuator(nh, rt_nh);

  auto rt_thread = std::thread(
    [&]() {
      // sched_param sch;
      // sch.sched_priority = 90;
      //
      // if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
      //   throw std::runtime_error{std::string("failed to set scheduler: ") +
      //     std::strerror(errno)};
      // }

      // Spin the RT queue with 3 threads
      // (equivalent to ROS2 MultiThreadedExecutor with 3 threads)
      ros::AsyncSpinner rt_spinner(3, &rt_queue);
      rt_spinner.start();
      ros::waitForShutdown();
  });

  ros::spin();

  rt_thread.join();

  return 0;
}
