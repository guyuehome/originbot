import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class VlpR(Node):
    def __init__(self, dbnet_path, crnn_path) -> None:
        super().__init__('vlpr_node')
        # 导入所需要的包
        from vlpt_package.utils.utils import strLabelConverter
        from vlpt_package.crnn.crnn_infer import crnn_model
        from vlpt_package.dbnet.dbnet_infer import dbnet_model
        
        self.get_logger().info("models are loading...")
        # dbnet初始化
        self.dbnet = dbnet_model(dbnet_path)
        # crnn初始化
        alphabet = '0123456789abcdefghijklmnopqrstuvwxyz'
        converter = strLabelConverter(alphabet) 
        self.crnn = crnn_model(crnn_path, converter)

        self.get_logger().info("models init successfully")
        
        self.bridge = CvBridge()
        self.vlp_image_msg = Image()
        self.image_sub = self.create_subscription(Image,"/vlpr_node/image_sub",self.image_callback,10)
        self.vlp_image_pub = self.create_publisher(Image, "/vlp_image",10)
    
    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data)
            self.vlp_recognize(image)
            self.vlp_image_pub.publish(self.vlp_image_msg)
        except CvBridgeError as e:
            print(e)
        
    def vlp_recognize(self, image):
        # 识别字符轮廓
        contours, boxes_list, t = self.dbnet.predict(image)
        mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        cv2.drawContours(mask, contours, -1, (255), thickness=cv2.FILLED)
        kernel_dilate = np.ones((20, 20), np.uint8)
        kernel_erode = np.ones((10, 10), np.uint8)
        mask = cv2.erode(cv2.dilate(mask, kernel_dilate), kernel_erode)
        
        # 通过字符轮廓得到车牌轮廓
        vlp_contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        vlp_cnt_length = len(vlp_contours)
        if vlp_cnt_length:
            # 找到最大轮廓
            area = []
            for k in range(vlp_cnt_length):
                area.append(cv2.contourArea(vlp_contours[k]))
            max_idx = np.argmax(np.array(area)) if len(area) else 0
            del area
            
            cnt_len = cv2.arcLength(vlp_contours[max_idx], True)
            approx_coefficient = 0.01
            while (True):
                cnt = cv2.approxPolyDP(vlp_contours[max_idx], approx_coefficient*cnt_len, True)
                if len(cnt) <= 4:
                    break
                else:
                    approx_coefficient = approx_coefficient + 0.005
                    continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            crnn_infer_img = cv2.cvtColor(image[y:y+h,x:x+w,:],cv2.COLOR_BGR2GRAY)
            raw_pred,sim_pred = self.crnn.predict(crnn_infer_img)
            result = ''.join(c for c in sim_pred if c.isdigit())
            if len(result) == 4:
                self.get_logger().info('result: OB·%s' % result)

            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            self.vlp_image_msg = self.bridge.cv2_to_imgmsg(image, encoding = "bgr8")

        else:
            self.vlp_image_msg = self.bridge.cv2_to_imgmsg(image, encoding = "bgr8")
            self.get_logger().info("can't find License Plate!")

    def bgr2nv12_opencv(self, image):
        height, width = image.shape[0], image.shape[1]
        area = height * width
        yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

        nv12 = np.zeros_like(yuv420p)
        nv12[:height * width] = y
        nv12[height * width:] = uv_packed
        return nv12
            
# CRNN_PATH = 'vlpr/vlpr/vlpt_package/crnn/crnn_simp.bin'
# DBNET_PATH = 'vlpr/vlpr/vlpt_package/dbnet/dbnet_simp.bin'
import pkg_resources
CRNN_PATH = pkg_resources.resource_filename('vlpt_package.crnn', 'crnn_simp.bin')
DBNET_PATH = pkg_resources.resource_filename('vlpt_package.dbnet', 'dbnet_simp.bin')

def main(args=None):
    rclpy.init(args=args)
    node = VlpR(DBNET_PATH, CRNN_PATH)
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
