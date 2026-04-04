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

#include <string>
#include <list>
#include <memory>
#include <vector>
#include <set>
#include <thread>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "br2_bt_patrolling/TrackObjects.hpp"
#include "br2_bt_patrolling/ctrl_support/BTLifecycleCtrlNode.hpp"

#include "gtest/gtest.h"


class VelocitySinkNode
{
public:
  VelocitySinkNode()
  {
    vel_sub_ = nh_.subscribe(
      "/output_vel", 100, &VelocitySinkNode::vel_callback, this);
  }

  void vel_callback(const geometry_msgs::Twist::ConstPtr & msg)
  {
    vel_msgs_.push_back(*msg);
  }

  std::list<geometry_msgs::Twist> vel_msgs_;

private:
  ros::NodeHandle nh_;
  ros::Subscriber vel_sub_;
};

class Nav2FakeServer
{
  using MoveBase = move_base_msgs::MoveBaseAction;

public:
  Nav2FakeServer()
  : as_(nh_, "move_base", boost::bind(&Nav2FakeServer::execute, this, _1), false)
  {}

  void start_server()
  {
    as_.start();
  }

private:
  void execute(const move_base_msgs::MoveBaseGoalConstPtr & /*goal*/)
  {
    move_base_msgs::MoveBaseFeedback feedback;

    auto start = ros::Time::now();
    ros::Rate rate(10);

    while ((ros::Time::now() - start).toSec() < 5.0) {
      if (as_.isPreemptRequested() || !ros::ok()) {
        as_.setPreempted();
        return;
      }
      as_.publishFeedback(feedback);
      rate.sleep();
    }

    as_.setSucceeded(move_base_msgs::MoveBaseResult());
  }

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<MoveBase> as_;
};

// Mock lifecycle node for ROS1 that emulates ROS2 lifecycle behavior
// using std_srvs services
class MockLifecycleNode
{
public:
  enum State { UNCONFIGURED = 0, INACTIVE = 1, ACTIVE = 2 };

  explicit MockLifecycleNode(const std::string & name)
  : state_(UNCONFIGURED), name_(name)
  {
    set_active_srv_ = nh_.advertiseService(
      name + "/set_active", &MockLifecycleNode::setActiveCb, this);
    get_state_srv_ = nh_.advertiseService(
      name + "/get_state", &MockLifecycleNode::getStateCb, this);
  }

  void configure()
  {
    if (state_ == UNCONFIGURED) {
      state_ = INACTIVE;
    }
  }

  State getState() const { return state_; }

private:
  bool setActiveCb(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
  {
    if (req.data) {
      // activate
      if (state_ == INACTIVE) {
        state_ = ACTIVE;
        res.success = true;
        res.message = "Activated";
      } else {
        res.success = false;
        res.message = "Cannot activate from current state";
      }
    } else {
      // deactivate
      if (state_ == ACTIVE) {
        state_ = INACTIVE;
        res.success = true;
        res.message = "Deactivated";
      } else {
        res.success = false;
        res.message = "Cannot deactivate from current state";
      }
    }
    return true;
  }

  bool getStateCb(std_srvs::Trigger::Request & /*req*/, std_srvs::Trigger::Response & res)
  {
    res.success = (state_ == ACTIVE);
    res.message = std::to_string(static_cast<int>(state_));
    return true;
  }

  State state_;
  std::string name_;
  ros::NodeHandle nh_;
  ros::ServiceServer set_active_srv_;
  ros::ServiceServer get_state_srv_;
};

class StoreWP : public BT::ActionNodeBase
{
public:
  explicit StoreWP(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf) {}

  void halt() {}
  BT::NodeStatus tick()
  {
    waypoints_.push_back(getInput<geometry_msgs::PoseStamped>("in").value());
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
    {
      BT::InputPort<geometry_msgs::PoseStamped>("in")
    });
  }

  static std::vector<geometry_msgs::PoseStamped> waypoints_;
};

std::vector<geometry_msgs::PoseStamped> StoreWP::waypoints_;

TEST(bt_action, recharge_btn)
{
  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_recharge_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Recharge    name="recharge"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);

  bool finish = false;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rate.sleep();
  }

  float battery_level;
  ASSERT_TRUE(blackboard->get("battery_level", battery_level));
  ASSERT_NEAR(battery_level, 100.0f, 0.0000001);
}

TEST(bt_action, patrol_btn)
{
  ros::NodeHandle nh;
  auto node_sink = std::make_shared<VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Patrol    name="patrol"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);

  bool finish = false;
  int counter = 0;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 150, 2);

  geometry_msgs::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.angular.z, 0.1);
  ASSERT_NEAR(one_twist.linear.x, 0.0, 0.0000001);
}

TEST(bt_action, move_btn)
{
  ros::NodeHandle nh;
  auto nav2_fake_node = std::make_shared<Nav2FakeServer>();
  nav2_fake_node->start_server();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Move    name="move" goal="{goal}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);

  geometry_msgs::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);

  bool finish = false;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rate.sleep();
  }

  spinner.stop();
}

TEST(bt_action, get_waypoint_btn)
{
  ros::NodeHandle nh;

  ros::spinOnce();

  {
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("br2_get_waypoint_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GetWaypoint    name="recharge" wp_id="{id}" waypoint="{waypoint}"/>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", nh);
    blackboard->set<std::string>("id", "recharge");

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    ros::Rate rate(10);

    bool finish = false;
    int counter = 0;
    while (!finish && ros::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      counter++;
      rate.sleep();
    }

    auto point = blackboard->get<geometry_msgs::PoseStamped>("waypoint");

    ASSERT_EQ(counter, 1);
    ASSERT_NEAR(point.pose.position.x, -1.0, 0.0000001);
    ASSERT_NEAR(point.pose.position.y, 4.14, 0.0000001);
  }

  {
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerNodeType<StoreWP>("StoreWP");
    factory.registerFromPlugin(loader.getOSName("br2_get_waypoint_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <Sequence name="root_sequence">
             <GetWaypoint    name="wp1" wp_id="next" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp2" wp_id="next" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp3" wp_id="" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp4" wp_id="recharge" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp5" wp_id="wp1" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp6" wp_id="wp2" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wpt" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
          </Sequence>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", nh);

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    ros::Rate rate(10);

    bool finish = false;
    while (!finish && ros::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      rate.sleep();
    }

    const auto & waypoints = StoreWP::waypoints_;
    ASSERT_EQ(waypoints.size(), 7);
    ASSERT_NEAR(waypoints[0].pose.position.x, 6.7, 0.0000001);
    ASSERT_NEAR(waypoints[0].pose.position.y, -2.54, 0.0000001);
    ASSERT_NEAR(waypoints[1].pose.position.x, 0.45, 0.0000001);
    ASSERT_NEAR(waypoints[1].pose.position.y, -3.9, 0.0000001);
    ASSERT_NEAR(waypoints[2].pose.position.x, -6.01, 0.0000001);
    ASSERT_NEAR(waypoints[2].pose.position.y, -0.3, 0.0000001);

    ASSERT_NEAR(waypoints[3].pose.position.x, -1.0, 0.0000001);
    ASSERT_NEAR(waypoints[3].pose.position.y, 4.14, 0.0000001);

    ASSERT_NEAR(waypoints[4].pose.position.x, 6.7, 0.0000001);
    ASSERT_NEAR(waypoints[4].pose.position.y, -2.54, 0.0000001);
    ASSERT_NEAR(waypoints[5].pose.position.x, 0.45, 0.0000001);
    ASSERT_NEAR(waypoints[5].pose.position.y, -3.9, 0.0000001);
    ASSERT_NEAR(waypoints[6].pose.position.x, -6.01, 0.0000001);
    ASSERT_NEAR(waypoints[6].pose.position.y, -0.3, 0.0000001);
  }
}

TEST(bt_action, battery_checker_btn)
{
  ros::NodeHandle nh;
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/output_vel", 100);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_battery_checker_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <ReactiveSequence>
              <BatteryChecker    name="battery_checker"/>
              <Patrol    name="patrol"/>
          </ReactiveSequence>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ros::Rate rate(10);
  geometry_msgs::Twist vel;
  vel.linear.x = 0.8;

  bool finish = false;
  int counter = 0;
  while (!finish && ros::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    vel_pub.publish(vel);

    ros::spinOnce();
    rate.sleep();
  }

  float battery_level;
  ASSERT_TRUE(blackboard->get("battery_level", battery_level));
  ASSERT_NEAR(battery_level, 94.6, 1.0);
}

TEST(bt_action, track_objects_btn_1)
{
  ros::NodeHandle nh;
  auto mock_head_tracker = std::make_shared<MockLifecycleNode>("head_tracker");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::NodeConfiguration conf;
  conf.blackboard = BT::Blackboard::create();
  conf.blackboard->set("node", nh);
  br2_bt_patrolling::BtLifecycleCtrlNode bt_node("TrackObjects", "head_tracker", conf);

  bt_node.set_active_client_ = bt_node.createServiceClient<std_srvs::SetBool>(
    "/head_tracker/set_active");
  ASSERT_TRUE(bt_node.set_active_client_.exists());

  bt_node.get_state_client_ = bt_node.createServiceClient<std_srvs::Trigger>(
    "/head_tracker/get_state");
  ASSERT_TRUE(bt_node.get_state_client_.exists());

  auto start = ros::Time::now();
  ros::Rate rate(10);
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  ASSERT_EQ(bt_node.get_state(), br2_bt_patrolling::STATE_UNCONFIGURED);
  bt_node.ctrl_node_state_ = br2_bt_patrolling::STATE_UNCONFIGURED;
  ASSERT_FALSE(bt_node.set_state(br2_bt_patrolling::STATE_ACTIVE));

  mock_head_tracker->configure();

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  bt_node.ctrl_node_state_ = bt_node.get_state();

  ASSERT_TRUE(bt_node.set_state(br2_bt_patrolling::STATE_ACTIVE));
  ASSERT_EQ(bt_node.get_state(), br2_bt_patrolling::STATE_ACTIVE);

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  bt_node.ctrl_node_state_ = bt_node.get_state();

  ASSERT_TRUE(bt_node.set_state(br2_bt_patrolling::STATE_INACTIVE));
  ASSERT_EQ(bt_node.get_state(), br2_bt_patrolling::STATE_INACTIVE);

  spinner.stop();
}

TEST(bt_action, track_objects_btn_2)
{
  ros::NodeHandle nh;
  auto mock_head_tracker = std::make_shared<MockLifecycleNode>("head_tracker");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::NodeConfiguration conf;
  conf.blackboard = BT::Blackboard::create();
  conf.blackboard->set("node", nh);
  br2_bt_patrolling::BtLifecycleCtrlNode bt_node("TrackObjects", "head_tracker", conf);

  mock_head_tracker->configure();

  ros::Rate rate(10);
  auto start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  ASSERT_EQ(bt_node.tick(), BT::NodeStatus::RUNNING);

  ASSERT_TRUE(bt_node.set_active_client_.exists());
  ASSERT_TRUE(bt_node.get_state_client_.exists());

  ASSERT_EQ(bt_node.get_state(), br2_bt_patrolling::STATE_ACTIVE);

  ASSERT_EQ(bt_node.tick(), BT::NodeStatus::RUNNING);

  bt_node.halt();

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  ASSERT_EQ(bt_node.get_state(), br2_bt_patrolling::STATE_INACTIVE);

  spinner.stop();
}

TEST(bt_action, track_objects_btn_3)
{
  ros::NodeHandle nh;
  auto mock_head_tracker = std::make_shared<MockLifecycleNode>("head_tracker");

  mock_head_tracker->configure();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_track_objects_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <KeepRunningUntilFailure>
              <TrackObjects    name="track_objects"/>
          </KeepRunningUntilFailure>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);
  auto start = ros::Time::now();
  ros::Rate rate(10);

  {
    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    ASSERT_EQ(
      static_cast<int>(mock_head_tracker->getState()),
      static_cast<int>(MockLifecycleNode::INACTIVE));

    while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
      tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING;

      rate.sleep();
    }
    ASSERT_EQ(
      static_cast<int>(mock_head_tracker->getState()),
      static_cast<int>(MockLifecycleNode::ACTIVE));
  }

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  ASSERT_EQ(
    static_cast<int>(mock_head_tracker->getState()),
    static_cast<int>(MockLifecycleNode::INACTIVE));

  spinner.stop();
}

TEST(bt_action, move_track_btn)
{
  ros::NodeHandle nh;
  auto nav2_fake_node = std::make_shared<Nav2FakeServer>();
  auto mock_head_tracker = std::make_shared<MockLifecycleNode>("head_tracker");

  mock_head_tracker->configure();
  nav2_fake_node->start_server();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_track_objects_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Parallel success_threshold="1" failure_threshold="1">
            <TrackObjects    name="track_objects"/>
            <Move    name="move" goal="{goal}"/>
          </Parallel>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", nh);

  geometry_msgs::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  ASSERT_EQ(
    static_cast<int>(mock_head_tracker->getState()),
    static_cast<int>(MockLifecycleNode::INACTIVE));

  ros::Rate rate(10);
  auto start = ros::Time::now();
  auto finish_tree = false;
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    finish_tree = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rate.sleep();
  }

  ASSERT_FALSE(finish_tree);
  ASSERT_EQ(
    static_cast<int>(mock_head_tracker->getState()),
    static_cast<int>(MockLifecycleNode::ACTIVE));

  while (ros::ok() && !finish_tree) {
    finish_tree = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rate.sleep();
  }

  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start).toSec() < 1.0) {
    rate.sleep();
  }

  ASSERT_EQ(
    static_cast<int>(mock_head_tracker->getState()),
    static_cast<int>(MockLifecycleNode::INACTIVE));

  spinner.stop();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bt_action_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
