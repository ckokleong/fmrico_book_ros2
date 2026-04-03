# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class BumpGoNode(object):

    def __init__(self):
        self.FORWARD = 0
        self.BACK = 1
        self.TURN = 2
        self.STOP = 3
        self.state = self.FORWARD
        self.state_ts = rospy.Time.now()

        self.TURNING_TIME = 2.0
        self.BACKING_TIME = 2.0
        self.SCAN_TIMEOUT = 1.0

        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.3
        self.OBSTACLE_DISTANCE = 1.0

        self.last_scan = None

        self.scan_sub = rospy.Subscriber(
            'input_scan', LaserScan, self.scan_callback, queue_size=1)

        self.vel_pub = rospy.Publisher('output_vel', Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_cycle)

    def scan_callback(self, msg):
        self.last_scan = msg

    def control_cycle(self, event):
        if self.last_scan is None:
            return

        out_vel = Twist()

        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR

            if self.check_forward_2_stop():
                self.go_state(self.STOP)
            if self.check_forward_2_back():
                self.go_state(self.BACK)

        elif self.state == self.BACK:
            out_vel.linear.x = -self.SPEED_LINEAR

            if self.check_back_2_turn():
                self.go_state(self.TURN)

        elif self.state == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR

            if self.check_turn_2_forward():
                self.go_state(self.FORWARD)

        elif self.state == self.STOP:
            if self.check_stop_2_forward():
                self.go_state(self.FORWARD)

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = rospy.Time.now()

    def check_forward_2_back(self):
        pos = len(self.last_scan.ranges) // 2
        return self.last_scan.ranges[pos] < self.OBSTACLE_DISTANCE

    def check_forward_2_stop(self):
        elapsed = rospy.Time.now() - self.last_scan.header.stamp
        return elapsed > rospy.Duration(self.SCAN_TIMEOUT)

    def check_stop_2_forward(self):
        elapsed = rospy.Time.now() - self.last_scan.header.stamp
        return elapsed < rospy.Duration(self.SCAN_TIMEOUT)

    def check_back_2_turn(self):
        elapsed = rospy.Time.now() - self.state_ts
        return elapsed > rospy.Duration(self.BACKING_TIME)

    def check_turn_2_forward(self):
        elapsed = rospy.Time.now() - self.state_ts
        return elapsed > rospy.Duration(self.TURNING_TIME)


def main():
    rospy.init_node('bump_go')
    bump_go_node = BumpGoNode()
    rospy.spin()


if __name__ == '__main__':
    main()
