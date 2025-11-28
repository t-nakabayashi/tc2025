#!/usr/bin/env python3

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraSimulatorNode(Node):
    """固定フレーム画像を指定トピックへ配信するシミュレータノード。"""

    def __init__(self) -> None:
        super().__init__('camera_simulator_node')

        self.declare_parameter('frame_image_path', '')
        self.declare_parameter('frame_width', -1)
        self.declare_parameter('frame_height', -1)
        self.declare_parameter('frame_ratio', 10.0)

        self.frame_path = self.get_parameter('frame_image_path').value
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.frame_rate = float(self.get_parameter('frame_ratio').value)

        self.bridge = CvBridge()
        self.prepared_frame: Optional[np.ndarray] = None

        self.image_publisher = self.create_publisher(Image, '/usb_cam/image_raw', 10)
        self.create_subscription(String, '/frame_image_path', self.path_callback, 10)

        self._configure_frame(self.frame_path)

        if self.frame_rate > 0:
            timer_period = 1.0 / self.frame_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(
                f'画像を{self.frame_rate:.2f}Hzで/usb_cam/image_rawに配信します。'
            )
        else:
            self.timer = None
            self.get_logger().warn('frame_ratioが0以下のためpublishを無効化しました。')

        self.get_logger().info('CameraSimulatorNodeを起動しました。')

    def path_callback(self, msg: String) -> None:
        """外部からパスを受信し、フレーム画像を差し替える。"""
        path = msg.data.strip()
        if not path:
            self.get_logger().warn('空のパスを受信したため無視しました。')
            return

        if self._configure_frame(path):
            self.get_logger().info(f'新しいフレーム画像に切り替えました: {path}')
        else:
            self.get_logger().warn(f'フレーム画像の読み込みに失敗しました: {path}')

    def timer_callback(self) -> None:
        """指定周期でフレームをpublishする。"""
        if self.prepared_frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(self.prepared_frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(msg)

    def _configure_frame(self, path: str) -> bool:
        """フレーム画像の読み込みとリサイズを行い、publish用データを準備する。"""
        if not path:
            self.prepared_frame = None
            self.get_logger().warn('フレーム画像パスが未指定のためpublishを停止します。')
            return False

        image = cv2.imread(path)
        if image is None:
            self.prepared_frame = None
            self.get_logger().warn(f'静止画の読み込みに失敗しました: {path}')
            return False

        resized = self._resize_image(image)
        if resized is None:
            self.prepared_frame = image
        else:
            self.prepared_frame = resized

        return True

    def _resize_image(self, image: np.ndarray) -> Optional[np.ndarray]:
        """パラメータに応じてアスペクト比を保持したリサイズを実施する。"""
        if self.frame_width < 0 and self.frame_height < 0:
            return None

        height, width = image.shape[:2]

        if self.frame_width > 0 and self.frame_height > 0:
            return self._resize_with_letterbox(image, width, height)

        if self.frame_width > 0:
            new_width = self.frame_width
            scale = new_width / width
            new_height = int(round(height * scale))
            return cv2.resize(image, (new_width, new_height))

        new_height = self.frame_height
        scale = new_height / height
        new_width = int(round(width * scale))
        return cv2.resize(image, (new_width, new_height))

    def _resize_with_letterbox(
        self, image: np.ndarray, width: int, height: int
    ) -> np.ndarray:
        """指定サイズに収めつつレターボックスを付与する。"""
        target_w = self.frame_width
        target_h = self.frame_height

        scale = min(target_w / width, target_h / height)
        resized_w = int(round(width * scale))
        resized_h = int(round(height * scale))

        resized_image = cv2.resize(image, (resized_w, resized_h))
        letterboxed = np.zeros((target_h, target_w, 3), dtype=image.dtype)

        x_offset = (target_w - resized_w) // 2
        y_offset = (target_h - resized_h) // 2
        letterboxed[y_offset:y_offset + resized_h, x_offset:x_offset + resized_w] = (
            resized_image
        )

        return letterboxed


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = CameraSimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
