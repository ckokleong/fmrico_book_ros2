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

#include <limits>
#include <vector>

#include "sensor_msgs/LaserScan.h"
#include "br2_vff_avoidance/AvoidanceNode.hpp"

#include "ros/ros.h"
#include "gtest/gtest.h"

class AvoidanceNodeTest : public br2_vff_avoidance::AvoidanceNode
{
public:
  br2_vff_avoidance::VFFVectors
  get_vff_test(const sensor_msgs::LaserScan & scan)
  {
    return get_vff(scan);
  }

  visualization_msgs::MarkerArray
  get_debug_vff_test(const br2_vff_avoidance::VFFVectors & vff_vectors)
  {
    return get_debug_vff(vff_vectors);
  }
};

sensor_msgs::LaserScan get_scan_test_1(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::LaserScan get_scan_test_2(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::LaserScan get_scan_test_3(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[2] = 0.3;

  return ret;
}

sensor_msgs::LaserScan get_scan_test_4(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[6] = 0.3;

  return ret;
}

sensor_msgs::LaserScan get_scan_test_5(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::LaserScan get_scan_test_6(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.5);
  ret.ranges[10] = 0.3;

  return ret;
}

sensor_msgs::LaserScan get_scan_test_7(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[14] = 0.3;

  return ret;
}

sensor_msgs::LaserScan get_scan_test_8(ros::Time ts)
{
  sensor_msgs::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[8] = 0.01;

  return ret;
}

TEST(vff_tests, get_vff)
{
  AvoidanceNodeTest node_avoidance;

  ros::Time ts = ros::Time::now();

  auto res1 = node_avoidance.get_vff_test(get_scan_test_1(ts));
  ASSERT_EQ(res1.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_EQ(res1.repulsive, std::vector<float>({0.0f, 0.0f}));
  ASSERT_EQ(res1.result, std::vector<float>({1.0f, 0.0f}));

  auto res2 = node_avoidance.get_vff_test(get_scan_test_2(ts));
  ASSERT_EQ(res2.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_NEAR(res2.repulsive[0], 1.0f, 0.00001f);
  ASSERT_NEAR(res2.repulsive[1], 0.0f, 0.00001f);
  ASSERT_NEAR(res2.result[0], 2.0f, 0.00001f);
  ASSERT_NEAR(res2.result[1], 0.0f, 0.00001f);

  auto res3 = node_avoidance.get_vff_test(get_scan_test_3(ts));
  ASSERT_EQ(res3.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_GT(res3.repulsive[0], 0.0f);
  ASSERT_GT(res3.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res3.repulsive[1], res3.repulsive[0]), 0.1);
  ASSERT_LT(atan2(res3.repulsive[1], res3.repulsive[0]), M_PI_2);
  ASSERT_GT(atan2(res3.result[1], res3.result[0]), 0.1);
  ASSERT_LT(atan2(res3.result[1], res3.result[0]), M_PI_2);

  auto res4 = node_avoidance.get_vff_test(get_scan_test_4(ts));
  ASSERT_EQ(res4.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_LT(res4.repulsive[0], 0.0f);
  ASSERT_GT(res4.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res4.repulsive[1], res4.repulsive[0]), M_PI_2);
  ASSERT_LT(atan2(res4.repulsive[1], res4.repulsive[0]), M_PI);
  ASSERT_GT(atan2(res4.result[1], res4.result[0]), 0.0);
  ASSERT_LT(atan2(res4.result[1], res4.result[0]), M_PI_2);

  auto res5 = node_avoidance.get_vff_test(get_scan_test_5(ts));
  ASSERT_EQ(res5.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_LT(res5.repulsive[0], 0.0f);
  ASSERT_LT(res5.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res5.repulsive[1], res5.repulsive[0]), -M_PI);
  ASSERT_LT(atan2(res5.repulsive[1], res5.repulsive[0]), -M_PI_2);
  ASSERT_LT(atan2(res5.result[1], res5.result[0]), 0.0);
  ASSERT_GT(atan2(res5.result[1], res5.result[0]), -M_PI_2);

  auto res6 = node_avoidance.get_vff_test(get_scan_test_6(ts));
  ASSERT_EQ(res6.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_LT(res6.repulsive[0], 0.0f);
  ASSERT_LT(res6.repulsive[1], 0.0f);
  ASSERT_GT(atan2(res6.repulsive[1], res6.repulsive[0]), -M_PI);
  ASSERT_LT(atan2(res6.repulsive[1], res6.repulsive[0]), -M_PI_2);
  ASSERT_LT(atan2(res6.result[1], res6.result[0]), 0.0);
  ASSERT_GT(atan2(res6.result[1], res6.result[0]), -M_PI_2);

  auto res7 = node_avoidance.get_vff_test(get_scan_test_7(ts));
  ASSERT_EQ(res7.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_GT(res7.repulsive[0], 0.0f);
  ASSERT_LT(res7.repulsive[1], 0.0f);
  ASSERT_LT(atan2(res7.repulsive[1], res7.repulsive[0]), 0.0f);
  ASSERT_GT(atan2(res7.repulsive[1], res7.repulsive[0]), -M_PI_2);
  ASSERT_LT(atan2(res7.result[1], res7.result[0]), 0.0);
  ASSERT_GT(atan2(res7.result[1], res7.result[0]), -M_PI_2);

  auto res8 = node_avoidance.get_vff_test(get_scan_test_8(ts));
  ASSERT_EQ(res8.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_NEAR(res8.repulsive[0], -1.0f, 0.1f);
  ASSERT_NEAR(res8.repulsive[1], 0.0f, 0.0001f);
  ASSERT_NEAR(res8.result[0], 0.0f, 0.01f);
  ASSERT_NEAR(res8.result[1], 0.0f, 0.01f);
}

TEST(vff_tests, ouput_vels)
{
  AvoidanceNodeTest node_avoidance;

  // Create a testing node with a scan publisher and a speed subscriber
  ros::NodeHandle nh;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("input_scan", 100);

  geometry_msgs::Twist last_vel;
  ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>(
    "output_vel", 1, [&last_vel](const geometry_msgs::Twist::ConstPtr & msg) {
      last_vel = *msg;
    });

  // Wait for connections to establish
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  ASSERT_EQ(vel_sub.getNumPublishers(), 1u);
  ASSERT_EQ(scan_pub.getNumSubscribers(), 1u);

  ros::Rate rate(30);

  // Test for scan test #1
  ros::Time start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_1(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.0001f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.0001f);

  // Test for scan test #2
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_2(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.3f, 0.0001f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.0001f);

  // Test for scan test #3
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_3(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_GT(last_vel.angular.z, 0.0f);
  ASSERT_LT(last_vel.angular.z, M_PI_2);

  // Test for scan test #4
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_4(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_GT(last_vel.angular.z, 0.0f);
  ASSERT_LT(last_vel.angular.z, M_PI_2);

  // Test for scan test #5
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_5(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for scan test #6
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_6(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for scan test #7
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(1.0)) {
    scan_pub.publish(get_scan_test_7(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_LT(last_vel.linear.x, 0.3f);
  ASSERT_GT(last_vel.linear.x, 0.0f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for scan test #8
  start = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(2.0)) {
    scan_pub.publish(get_scan_test_8(ros::Time::now()));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.0f, 0.1f);
  ASSERT_LT(last_vel.angular.z, 0.0f);
  ASSERT_GT(last_vel.angular.z, -M_PI_2);

  // Test for stopping when scan is too old
  last_vel = geometry_msgs::Twist();
  while (ros::ok() && (ros::Time::now() - start) < ros::Duration(3.0)) {
    scan_pub.publish(get_scan_test_6(start));
    ros::spinOnce();
    rate.sleep();
  }
  ASSERT_NEAR(last_vel.linear.x, 0.0f, 0.01f);
  ASSERT_NEAR(last_vel.angular.z, 0.0f, 0.01f);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "vff_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
