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

#ifndef BR2_VFF_AVOIDANCE__AVOIDANCENODE_HPP_
#define BR2_VFF_AVOIDANCE__AVOIDANCENODE_HPP_

#include <vector>

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"

#include "ros/ros.h"

namespace br2_vff_avoidance
{

struct VFFVectors
{
  std::vector<float> attractive;
  std::vector<float> repulsive;
  std::vector<float> result;
};

typedef enum {RED, GREEN, BLUE, NUM_COLORS} VFFColor;

class AvoidanceNode
{
public:
  AvoidanceNode();

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr & msg);
  void control_cycle(const ros::TimerEvent & event);

protected:
  VFFVectors get_vff(const sensor_msgs::LaserScan & scan);

  visualization_msgs::MarkerArray get_debug_vff(const VFFVectors & vff_vectors);
  visualization_msgs::Marker make_marker(
    const std::vector<float> & vector, VFFColor vff_color);

private:
  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Publisher vff_debug_pub_;
  ros::Subscriber scan_sub_;
  ros::Timer timer_;

  sensor_msgs::LaserScan::ConstPtr last_scan_;
};

}  // namespace br2_vff_avoidance

#endif  // BR2_VFF_AVOIDANCE__AVOIDANCENODE_HPP_
