
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'image_raw', self.callback, 10)
        self.compressed_pub = self.create_publisher(CompressedImage, 'compressed_image', 10)
        self.bgr8_pub = self.create_publisher(Image, 'bgr8_image', 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        _, compressed = cv2.imencode('.jpg', cv_image, encode_param)

        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = 'jpeg'
        compressed_msg.data = np.array(compressed).tostring()

        self.compressed_pub.publish(compressed_msg)
        decompressed = cv2.imdecode(np.frombuffer(compressed_msg.data, np.uint8), cv2.IMREAD_COLOR)
        bgr8_msg = self.bridge.cv2_to_imgmsg(decompressed, encoding='bgr8')
        self.compressed_pub.publish(compressed_msg)
        self.bgr8_pub.publish(bgr8_msg)

def main(args=None):
    rclpy.init(args=args)
    compressor = ImageCompressor()
    rclpy.spin(compressor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()