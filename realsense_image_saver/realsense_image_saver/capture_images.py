#!/usr/bin/env python3

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSaver(Node):
    """Simple node that saves incoming ROS2 images to disk."""

    def __init__(self):
        super().__init__('image_saver')
        self.declare_parameter('image_topic', '/color/image_raw')
        self.declare_parameter('output_dir', 'images')
        self.declare_parameter('save_interval_sec', 0.0)

        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        self.output_dir = (
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        self.save_interval = (
            self.get_parameter('save_interval_sec').get_parameter_value().double_value
        )
        self.last_save_time = self.get_clock().now()

        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, image_topic, self.listener_callback, 10
        )
        self.frame_id = 0
        self.get_logger().info(f'Subscribed to {image_topic}; saving to {self.output_dir}')

    def listener_callback(self, msg: Image) -> None:
        try:
            now = self.get_clock().now()
            if (
                self.save_interval > 0
                and (now - self.last_save_time).nanoseconds
                < self.save_interval * 1e9
            ):
                return
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.output_dir, f"frame_{self.frame_id:06d}.png")
            cv2.imwrite(filename, cv_image)
            self.get_logger().debug(f'Saved image {filename}')
            self.frame_id += 1
            self.last_save_time = now
        except Exception as exc:
            self.get_logger().error(f'Failed to save image: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
