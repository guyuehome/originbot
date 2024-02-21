#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

import time

X_CENTER_MAX = 380
X_CENTER_MIN = 270

Z_SIZE_MAX = 3500
Z_SIZE_MIN = 20000

X_CENTER_AVERAGE = (X_CENTER_MAX + X_CENTER_MIN) / 2
Z_SIZE_AVERAGE = (Z_SIZE_MAX + Z_SIZE_MIN) / 2


class QrCodeControl(Node):
    def __init__(self):
        super().__init__('qrcode_control')

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_qrcode_info = self.create_subscription(
            String, "/qrcode_detected/info_result", self.handleQrcodeInfo, 10)
        self.sub_qrcode_pose = self.create_subscription(
            Pose, "/qrcode_detected/pose_result", self.handleQrcodePose, 1)

        self.twist = Twist()
        self.declare_parameter("control_with_qrcode_info", True)
        self.control_with_qrcode_info = True

    def handleQrcodeInfo(self, qrcode_info: String):
        self.control_with_qrcode_info = self.get_parameter(
            'control_with_qrcode_info').get_parameter_value().bool_value
        if (self.control_with_qrcode_info):
            self.setTwistWithQrInfo(qrcode_info)

    def handleQrcodePose(self, qrcode_pose: Pose):
        self.control_with_qrcode_info = self.get_parameter(
            'control_with_qrcode_info').get_parameter_value().bool_value
        if (not self.control_with_qrcode_info):
            self.setTwistWithQrPose(qrcode_pose)

    def pubControlCommand(self):
        self.pub_cmd_vel.publish(self.twist)
        self.get_logger().info(
            "Publsh velocity command[{} m/s, {} rad/s]".format(
                self.twist.linear.x, self.twist.angular.z))
        time.sleep(0.03)      #未识别到二维码数据停止
        self.setTwist(0.0, 0.0)
        self.pub_cmd_vel.publish(self.twist)

    def setTwist(self, linear_x, angular_z):
        linear_x = max(min(linear_x, 0.1), -0.1)
        angular_z = max(min(angular_z, 1.0), -1.0)
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z

    def setTwistWithQrInfo(self, qrcode_info: String):
        info = qrcode_info.data
        if 'Front' in info:
            self.setTwist(0.1, 0.0)
        elif 'Back' in info:
            self.setTwist(-0.1, 0.0)
        elif 'Left' in info:
            self.setTwist(0.0, 0.4)
        elif 'Right' in info:
            self.setTwist(0.0, -0.4)
        else:
            self.setTwist(0.0, 0.0)
        
        self.pubControlCommand()

    def setTwistWithQrPose(self, qrcode_pose: Pose):
        pose = qrcode_pose
        qrcode_x = pose.position.x
        qrcode_size = pose.position.z
        
        self.setTwistWithX(qrcode_x)
        self.setTwistWithZ(qrcode_size)

        self.pubControlCommand()

    def setTwistWithX(self, qrcode_x):
        if qrcode_x > X_CENTER_MIN and qrcode_x < X_CENTER_MAX:
            self.setTwist(self.twist.linear.x, 0.0)
        elif qrcode_x < X_CENTER_MIN or qrcode_x > X_CENTER_MAX:
            self.setTwist(self.twist.linear.x, (1.0 - qrcode_x/X_CENTER_AVERAGE))
        else:
            self.get_logger().info("No X, cannot control!")

    def setTwistWithZ(self, qrcode_size):
        if qrcode_size >= Z_SIZE_MIN and qrcode_size <= Z_SIZE_MAX:
            self.setTwist(0.0, self.twist.angular.z)
        elif qrcode_size < Z_SIZE_MIN or qrcode_size > Z_SIZE_MAX:
            self.setTwist((1.0 - qrcode_size/Z_SIZE_AVERAGE) * 0.8, self.twist.angular.z)
        else:
            self.get_logger().info("No Z, cannot control!")


def main(args=None):
    rclpy.init(args=args)
    qrcode_control = QrCodeControl()
    while rclpy.ok():
        rclpy.spin(qrcode_control)

    qrcode_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
