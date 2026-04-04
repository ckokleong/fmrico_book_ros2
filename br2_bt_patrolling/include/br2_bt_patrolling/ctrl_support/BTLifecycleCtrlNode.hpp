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

#ifndef BR2_BT_PATROLLING__CTRL_SUPPORT__BTLIFECYCLECTRLNODE_HPP_
#define BR2_BT_PATROLLING__CTRL_SUPPORT__BTLIFECYCLECTRLNODE_HPP_

#include <memory>
#include <string>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>

namespace br2_bt_patrolling
{

// State constants to replace lifecycle_msgs::msg::State
static const uint8_t STATE_UNCONFIGURED = 0;
static const uint8_t STATE_INACTIVE = 1;
static const uint8_t STATE_ACTIVE = 2;
static const uint8_t STATE_UNKNOWN = 255;

class BtLifecycleCtrlNode : public BT::ActionNodeBase
{
public:
  BtLifecycleCtrlNode(
    const std::string & xml_tag_name,
    const std::string & node_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), ctrl_node_name_(node_name)
  {
    nh_ = config().blackboard->get<ros::NodeHandle>("node");
  }

  BtLifecycleCtrlNode() = delete;

  virtual ~BtLifecycleCtrlNode()
  {
  }

  template<typename ServiceT>
  ros::ServiceClient createServiceClient(const std::string & service_name)
  {
    auto srv = nh_.serviceClient<ServiceT>(service_name);
    while (!srv.waitForExistence(ros::Duration(1.0))) {
      if (!ros::ok()) {
        ROS_ERROR("Interrupted while waiting for the service. Exiting.");
        break;
      } else {
        ROS_INFO("service not available, waiting again...");
      }
    }
    return srv;
  }

  virtual void on_tick() {}

  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  virtual BT::NodeStatus on_failure()
  {
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus tick() override
  {
    if (status() == BT::NodeStatus::IDLE) {
      set_active_client_ = createServiceClient<std_srvs::SetBool>(
        ctrl_node_name_ + "/set_active");
      get_state_client_ = createServiceClient<std_srvs::Trigger>(
        ctrl_node_name_ + "/get_state");
    }

    if (ctrl_node_state_ != STATE_ACTIVE) {
      ctrl_node_state_ = get_state();
      set_state(STATE_ACTIVE);
    }

    on_tick();

    return BT::NodeStatus::RUNNING;
  }

  void halt() override
  {
    if (ctrl_node_state_ == STATE_ACTIVE) {
      set_state(STATE_INACTIVE);
    }
    setStatus(BT::NodeStatus::IDLE);
  }

  // Get the state of the controlled node
  uint8_t get_state()
  {
    std_srvs::Trigger srv;
    if (get_state_client_.call(srv)) {
      try {
        return static_cast<uint8_t>(std::stoi(srv.response.message));
      } catch (...) {
        ROS_ERROR("Failed to parse state from get_state service");
        return STATE_UNKNOWN;
      }
    }

    ROS_ERROR("Failed to call get_state service");
    return STATE_UNKNOWN;
  }

  // Set the state of the controlled node
  bool set_state(uint8_t state)
  {
    std_srvs::SetBool srv;

    if (state == STATE_ACTIVE &&
      ctrl_node_state_ == STATE_INACTIVE)
    {
      srv.request.data = true;
    } else {
      if (state == STATE_INACTIVE &&
        ctrl_node_state_ == STATE_ACTIVE)
      {
        srv.request.data = false;
      } else {
        if (state != ctrl_node_state_) {
          ROS_ERROR("Transition not possible %u -> %u", ctrl_node_state_, state);
          return false;
        } else {
          return true;
        }
      }
    }

    if (set_active_client_.call(srv)) {
      if (!srv.response.success) {
        ROS_ERROR("Failed to set node state %u -> %u", ctrl_node_state_, state);
        return false;
      } else {
        ROS_INFO("Transition success  %u -> %u", ctrl_node_state_, state);
      }
    } else {
      ROS_ERROR("Failed to call set_active service");
      return false;
    }

    ctrl_node_state_ = state;
    return true;
  }

  std::string ctrl_node_name_;
  uint8_t ctrl_node_state_;

  ros::ServiceClient set_active_client_;
  ros::ServiceClient get_state_client_;

  ros::NodeHandle nh_;
};


}  // namespace br2_bt_patrolling

#endif  // BR2_BT_PATROLLING__CTRL_SUPPORT__BTLIFECYCLECTRLNODE_HPP_
