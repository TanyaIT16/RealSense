#!/usr/bin/env python3

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

SAVE_INTERVAL_SEC = 30.0  # Intervalo fijo entre capturas


class ImageSaver(Node):
    """Nodo que guarda imágenes recibidas por un tópico de ROS2."""

    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_dir', 'images')

        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        self.output_dir = (
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        self.save_interval = SAVE_INTERVAL_SEC
        self.last_save_time = self.get_clock().now()

        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, image_topic, self.listener_callback, 10
        )

        self.get_logger().info(f'Subscrito a {image_topic}; guardando en {self.output_dir}')

    def listener_callback(self, msg: Image) -> None:
        try:
            now = self.get_clock().now()
            if (now - self.last_save_time).nanoseconds < self.save_interval * 1e9:
                return

            # Crear nombre de archivo con fecha y hora
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(self.output_dir, f"{timestamp}.png")

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(filename, cv_image)

            self.get_logger().info(f'Imagen guardada: {filename}')
            self.last_save_time = now

        except Exception as exc:
            self.get_logger().error(f'Error al guardar imagen: {exc}')


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
