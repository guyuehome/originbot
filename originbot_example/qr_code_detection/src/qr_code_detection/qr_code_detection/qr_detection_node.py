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
import cv2
import cv_bridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


modelPath = "/userdata/dev_ws/src/originbot/originbot_example/qr_code_detection/model/"

#预定义变量
color = (0, 0, 255)
thick = 3
font_scale = 0.5
font_thickness = 2

class QrCodeDetection(Node):
    def __init__(self):
        super().__init__('qrcode_detect')
        self.bridge = cv_bridge.CvBridge()

        #接受来自utils/NV122BGR的imgae_out
        self.image_sub = self.create_subscription(
            CompressedImage, "/image_out/compressed", self.image_callback, 10)

        self.pub_img = self.create_publisher(
            Image, '/qrcode_detected_result', 10)

        self.detect_obj = cv2.wechat_qrcode_WeChatQRCode(
            modelPath+'detect.prototxt', modelPath+'detect.caffemodel',
            modelPath+'sr.prototxt', modelPath+'sr.caffemodel')

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        qrInfo, qrPoints = self.detect_obj.detectAndDecode(cv_image)
        emptyList = ()
        if qrInfo != emptyList:
            self.get_logger().info('qrInfo: "{0}"'.format(qrInfo))
            self.get_logger().info('qrPoints: "{0}"'.format(qrPoints))

            for pos in qrPoints:
                for p in [(0, 1), (1, 2), (2, 3), (3, 0)]:
                    start = int(pos[p[0]][0]), int(pos[p[0]][1])
                    end = int(pos[p[1]][0]), int(pos[p[1]][1])
                    cv2.line(cv_image, start, end, color, thick)

                qrInfo_str = qrInfo[0]
                font = cv2.FONT_HERSHEY_SIMPLEX
                text_position = (int(pos[0][0]), int(pos[0][1]) - 10)

                cv2.putText(cv_image, qrInfo_str, 
                    text_position, font, font_scale, color, font_thickness)

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

def main(args=None):

    rclpy.init(args = args)

    qrCodeDetection = QrCodeDetection()
    while rclpy.ok():
        rclpy.spin(qrCodeDetection)

    qrCodeDetection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
