# Copyright (c) 2022，Horizon Robotics.
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
from cv_bridge import CvBridge
import cv2 as cv
import uuid
import os
import numpy as np

from sensor_msgs.msg import Image


class ImageSubscriber(Node):

  def __init__(self):
    super().__init__('ImageSubscriber')
    # 创建sub节点，订阅image_raw话题
    self.subscription = self.create_subscription(
      Image,
      'image_raw',
      self.listener_callback,
      1)
    # 创建CvBridge实例
    self.bridge = CvBridge()
    self.x = -1
    self.y = -1
    self.uuid = -1
    self.image = np.zeros((960, 224, 3))
    self.initialize = False

    # 检查用户存放标定数据的image_dataset文件夹是否存在，如果不存在则创建
    if not os.path.exists('./image_dataset'):
      os.makedirs('./image_dataset')

    # 设置opecv展示窗属性
    cv.namedWindow("capture image", cv.WINDOW_NORMAL)
    cv.resizeWindow("capture image", 960, 224)
    self.subscription
  
  # 鼠标事件，当左键按下则在对应坐标处画圈，表示标记的位置
  def mouse_callback(self, event, x, y, flags, userdata):
    if event == cv.EVENT_LBUTTONDOWN:
      imageWithCircle = userdata.copy()
      self.x = x
      self.y = y
      cv.circle(imageWithCircle, (x,y), 5, (0,0,255), -1)
      cv.imshow("capture image", imageWithCircle)

  # sub回调函数
  def listener_callback(self, msg):
    keyValue = cv.waitKey(1)
    # 检测到按下回车键，则获取一张新的图像
    if keyValue == 13:
      captureImage = self.bridge.imgmsg_to_cv2(msg)
      cropImage = captureImage[160:384,:,:].copy()
      if self.initialize == False:
        self.image = cropImage.copy()
        self.initialize = True
      # 注册鼠标回调
      cv.setMouseCallback("capture image", self.mouse_callback, cropImage)
      cv.imshow("capture image", cropImage)
      if self.x != -1 and self.y != -1:
        self.uuid = 'xy_%03d_%03d_%s' % (self.x, self.y, uuid.uuid1())
        # 保存上一次标定的结果
        cv.imwrite('./image_dataset/' + self.uuid + '.jpg', self.image)
        # 载入新的图像
        self.image = cropImage.copy()
        self.x = -1
        self.y = -1

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()