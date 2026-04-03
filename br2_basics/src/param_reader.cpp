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

#include <vector>
#include <string>

#include "ros/ros.h"

class LocalizationNode
{
public:
  LocalizationNode()
  : nh_("~")
  {
    nh_.param<int>("number_particles", num_particles_, 200);
    ROS_INFO_STREAM("Number of particles: " << num_particles_);

    nh_.getParam("topics", topics_);
    nh_.getParam("topic_types", topic_types_);

    if (topics_.size() != topic_types_.size()) {
      ROS_ERROR("Number of topics (%zu) != number of types (%zu)",
        topics_.size(), topic_types_.size());
    } else {
      ROS_INFO_STREAM("Number of topics: " << topics_.size());
      for (size_t i = 0; i < topics_.size(); i++) {
        ROS_INFO_STREAM("\t" << topics_[i] << "\t - " << topic_types_[i]);
      }
    }
  }

private:
  ros::NodeHandle nh_;
  int num_particles_;
  std::vector<std::string> topics_;
  std::vector<std::string> topic_types_;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "localization_node");

  LocalizationNode node;

  ros::spin();

  return 0;
}
