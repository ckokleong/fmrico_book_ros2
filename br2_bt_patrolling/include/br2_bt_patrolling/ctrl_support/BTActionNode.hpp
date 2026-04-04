// Copyright (c) 2018 Intel Corporation
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

#ifndef BR2_BT_PATROLLING__CTRL_SUPPORT__BTACTIONNODE_HPP_
#define BR2_BT_PATROLLING__CTRL_SUPPORT__BTACTIONNODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace br2_bt_patrolling
{

template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  using GoalType = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;
  using FeedbackType = typename ActionT::_action_feedback_type::_feedback_type;
  using ActionClientT = actionlib::SimpleActionClient<ActionT>;

  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name),
    result_state_(actionlib::SimpleClientGoalState::PENDING)
  {
    nh_ = config().blackboard->get<ros::NodeHandle>("node");

    // Initialize the goal message
    goal_ = GoalType();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);

    // Give the derived class a chance to do any initialization
    ROS_INFO("\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  // Create instance of an action client
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = std::make_shared<ActionClientT>(action_name, true);

    // Make sure the server is actually there before continuing
    ROS_INFO("Waiting for \"%s\" action server", action_name.c_str());
    action_client_->waitForServer();
  }

  // Any subclass of BtActionNode that accepts parameters must provide a providedPorts method
  // and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  // Could do dynamic checks, such as getting updates to values on the blackboard
  virtual void on_tick()
  {
  }

  // There can be many loop iterations per tick. Any opportunity to do something after
  // a timeout waiting for a result that hasn't been received yet
  virtual void on_wait_for_result()
  {
  }

  // Called upon successful completion of the action. A derived class can override this
  // method to put a value on the blackboard, for example.
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Called when the action is aborted. By default, the node will return FAILURE.
  // The user may override it to return another value, instead.
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  // Called when the action is cancelled. By default, the node will return SUCCESS.
  // The user may override it to return another value, instead.
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // The main override required by a BT action
  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      createActionClient(action_name_);

      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // user defined callback
      on_tick();

      on_new_goal_received();
    }

    // The following code corresponds to the "RUNNING" loop
    if (ros::ok() && !goal_result_available_) {
      // user defined callback. May modify the value of "goal_updated_"
      on_wait_for_result();

      auto goal_state = action_client_->getState();
      if (goal_updated_ && (goal_state == actionlib::SimpleClientGoalState::ACTIVE ||
        goal_state == actionlib::SimpleClientGoalState::PENDING))
      {
        goal_updated_ = false;
        on_new_goal_received();
      }

      ros::spinOnce();

      // check if, after invoking spinOnce(), we finally received the result
      if (!goal_result_available_) {
        // Yield this Action, returning RUNNING
        return BT::NodeStatus::RUNNING;
      }
    }

    if (result_state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return on_success();
    } else if (result_state_ == actionlib::SimpleClientGoalState::ABORTED) {
      return on_aborted();
    } else if (result_state_ == actionlib::SimpleClientGoalState::PREEMPTED ||
               result_state_ == actionlib::SimpleClientGoalState::RECALLED) {
      return on_cancelled();
    } else {
      throw std::logic_error("BtActionNode::Tick: invalid status value");
    }
  }

  // The other (optional) override required by a BT action. In this case, we
  // make sure to cancel the ROS action if it is still running.
  void halt() override
  {
    if (should_cancel_goal()) {
      action_client_->cancelGoal();
    }

    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    ros::spinOnce();
    auto state = action_client_->getState();

    // Check if the goal is still executing
    return state == actionlib::SimpleClientGoalState::ACTIVE ||
           state == actionlib::SimpleClientGoalState::PENDING;
  }


  void on_new_goal_received()
  {
    goal_result_available_ = false;
    action_client_->sendGoal(goal_,
      boost::bind(&BtActionNode::doneCb, this, _1, _2));
  }

  void doneCb(const actionlib::SimpleClientGoalState & state,
              const typename ResultType::ConstPtr & /*result*/)
  {
    goal_result_available_ = true;
    result_state_ = state;
  }

  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  std::shared_ptr<ActionClientT> action_client_;

  // All ROS actions have a goal and a result
  GoalType goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  actionlib::SimpleClientGoalState result_state_;

  // The node handle for ROS operations
  ros::NodeHandle nh_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;
};


}  // namespace br2_bt_patrolling

#endif  // BR2_BT_PATROLLING__CTRL_SUPPORT__BTACTIONNODE_HPP_
