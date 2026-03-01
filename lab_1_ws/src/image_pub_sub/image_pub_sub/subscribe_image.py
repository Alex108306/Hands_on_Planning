# Copyright 2016 Open Source Robotics Foundation, Inc.
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
from sensor_msgs.msg import Image, CompressedImage

import cv2
from cv_bridge import CvBridge


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.publisher_sobel = self.create_publisher(Image, 'image_mod_sobel', 10)
        self.publisher_laplacian = self.create_publisher(Image, 'image_mod_laplacian', 10)
        self.subscription_ = self.create_subscription(Image, 'image', self.extract_image, 10)
        self.subscription_


    def extract_image(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blur_image = cv2.GaussianBlur(cv_image, (3, 3), 1.0)
        gray = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)
        # Sobel filter
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient_magnitude = cv2.magnitude(sobelx, sobely)
        gradient_magnitude = cv2.convertScaleAbs(gradient_magnitude)
        # Laplacian filter
        laplacian = cv2.Laplacian(gray, cv2.CV_64F, ksize=3)
        laplacian_abs = cv2.convertScaleAbs(laplacian)
        ros_image_laplacian = bridge.cv2_to_imgmsg(laplacian_abs, encoding='mono8')
        ros_image_sobel = bridge.cv2_to_imgmsg(gradient_magnitude, encoding='mono8')
        # ros_compressed_image = bridge.cv2_to_compressed_imgmsg(filtered_image)
        self.publisher_sobel.publish(ros_image_sobel)
        self.publisher_laplacian.publish(ros_image_laplacian)
        

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
