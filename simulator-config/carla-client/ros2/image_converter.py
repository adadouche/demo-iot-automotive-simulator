import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import argparse
import sys

class ImageConverter(Node):
    def __init__(self, input_topic, output_topic):
        super().__init__('image_converter')
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            CompressedImage,
            output_topic,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        ret, jpeg_img = cv2.imencode('.jpg', cv_image)
        if ret:
            compressed_img = CompressedImage()
            compressed_img.header = data.header
            compressed_img.format = "jpeg"
            compressed_img.data = jpeg_img.tobytes()
            self.publisher.publish(compressed_img)

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 Image Compressor Node')
    parser.add_argument('--input_topic', type=str, default='raw_image_topic',
                        help='Input topic for raw images')
    parser.add_argument('--output_topic', type=str, default='compressed_image_topic',
                        help='Output topic for compressed images')

    args = parser.parse_args()

    rclpy.init(args=sys.argv[1:]) 
    image_converter = ImageConverter(input_topic=args.input_topic, output_topic=args.output_topic)
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
