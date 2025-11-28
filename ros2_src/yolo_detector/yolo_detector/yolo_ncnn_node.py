#!/usr/bin/env python3

import copy
import os
import threading
import time
from typing import List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class YoloNCNNDetectorNode(Node):
    """NCNN版YOLOで物体検出を行うノード。"""

    def __init__(self) -> None:
        super().__init__('yolo_ncnn_detector_node')

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('detection_interval', 0.5)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('class_names', ['item'])

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.detection_interval = (
            self.get_parameter('detection_interval').get_parameter_value().double_value
        )
        self.confidence_threshold = (
            self.get_parameter('confidence_threshold').get_parameter_value().double_value
        )
        self.class_names = (
            self.get_parameter('class_names').get_parameter_value().string_array_value
        )

        if model_path == '':
            default_model_path = os.path.join(
                get_package_share_directory('yolo_detector'),
                'models',
                'best_ncnn_model',
            )
            self.get_logger().warn(
                'model_pathが指定されていないため、パッケージ内のNCNNモデルを利用します: '
                f'{default_model_path}'
            )
            model_path = default_model_path

        self.model = self._load_model(model_path)
        self.bridge = CvBridge()

        self.latest_image: Optional[np.ndarray] = None
        self.latest_header: Optional[Header] = None
        self.image_lock = threading.Lock()
        self.timer_lock = threading.Lock()

        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
            callback_group=self.callback_group,
        )

        self.detection_image_publisher = self.create_publisher(
            Image, 'yolo_detector/image_det', 10
        )
        self.detection_publisher = self.create_publisher(
            Detection2DArray, 'yolo_detector/detections', 10
        )

        self.create_timer(
            self.detection_interval,
            self.timer_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(f'YoloNCNNDetectorNode started. Subscribing to {image_topic}')
        self.get_logger().info(f'Detection interval: {self.detection_interval} seconds')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Class names: {self.class_names}')

    def _load_model(self, model_path: str) -> YOLO:
        """NCNNモデルを読み込む。"""
        self.get_logger().info(f'Loading NCNN model from: {model_path}')
        try:
            model = YOLO(model_path, task='detect')
            self.get_logger().info('NCNN model loaded successfully')
            return model
        except Exception as exc:
            self.get_logger().error(f'Failed to load NCNN model: {exc}')
            raise

    def image_callback(self, msg: Image) -> None:
        """画像トピックのコールバック - 最新画像とヘッダを更新する。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            header_copy = copy.deepcopy(msg.header)
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        with self.image_lock:
            self.latest_image = cv_image
            self.latest_header = header_copy

    def timer_callback(self) -> None:
        """推論タイマーのコールバック。非ブロッキングで多重実行を防止する。"""
        if not self.timer_lock.acquire(blocking=False):
            return

        try:
            image_copy, header_copy = self._copy_latest_image()
            if image_copy is None or header_copy is None:
                return

            results, inference_time = self._perform_inference(image_copy)
            annotated_image, detection_array = self._create_outputs(results, header_copy)

            self.detection_publisher.publish(detection_array)

            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = header_copy
            self.detection_image_publisher.publish(annotated_msg)

            self._log_detection_results(results, inference_time)
        finally:
            self.timer_lock.release()

    def _copy_latest_image(self) -> Tuple[Optional[np.ndarray], Optional[Header]]:
        """最新画像とヘッダをディープコピーして返す。"""
        with self.image_lock:
            if self.latest_image is None or self.latest_header is None:
                return None, None

            return copy.deepcopy(self.latest_image), copy.deepcopy(self.latest_header)

    def _perform_inference(self, image: np.ndarray) -> Tuple[List, float]:
        """NCNNで推論を実行する。"""
        start_time = time.time()
        results = self.model(image, verbose=False, conf=self.confidence_threshold)
        inference_time = time.time() - start_time
        return results, inference_time

    def _create_outputs(
        self, results: List, header: Header
    ) -> Tuple[np.ndarray, Detection2DArray]:
        """検出結果から重畳画像とDetection2DArrayを生成する。"""
        annotated_image = results[0].orig_img.copy() if results else np.array([])

        detection_array = Detection2DArray()
        detection_array.header = header

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self._resolve_class_name(result.names, cls_id)
                confidence = float(box.conf[0])

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                detection_array.detections.append(
                    self._build_detection(
                        header, cls_id, class_name, confidence, x1, y1, x2, y2
                    )
                )

                annotated_image = self._draw_detection(
                    annotated_image, class_name, confidence, int(x1), int(y1), int(x2), int(y2)
                )

        return annotated_image, detection_array

    def _build_detection(
        self,
        header: Header,
        cls_id: int,
        class_name: str,
        confidence: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ) -> Detection2D:
        """単一検出結果をDetection2Dに変換する。"""
        detection = Detection2D()
        detection.header = header

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = str(cls_id)
        hypothesis.score = confidence
        detection.results.append(hypothesis)

        bbox = BoundingBox2D()
        bbox.center.x = (x1 + x2) / 2.0
        bbox.center.y = (y1 + y2) / 2.0
        bbox.center.theta = 0.0
        bbox.size_x = x2 - x1
        bbox.size_y = y2 - y1
        detection.bbox = bbox
        return detection

    def _draw_detection(
        self,
        image: np.ndarray,
        class_name: str,
        confidence: float,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> np.ndarray:
        """検出結果を画像に描画する。"""
        label = f'{class_name}:{confidence:.2f}'
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(
            image,
            label,
            (x1, max(y1 - 10, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )
        return image

    def _resolve_class_name(self, names: List[str], cls_id: int) -> str:
        """クラスIDからクラス名を取得する。"""
        try:
            if isinstance(names, dict):
                return names.get(cls_id, self._fallback_class_name(cls_id))
            return names[cls_id]
        except Exception:
            return self._fallback_class_name(cls_id)

    def _fallback_class_name(self, cls_id: int) -> str:
        """クラス名のフォールバックを返す。"""
        if cls_id < len(self.class_names):
            return self.class_names[cls_id]
        return f'class_{cls_id}'

    def _log_detection_results(self, results: List, inference_time: float) -> None:
        """検出結果をターミナルに出力する。"""
        fps = 1.0 / inference_time if inference_time > 0 else 0.0
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'Detection Results (Inference time: {inference_time:.3f}s = {fps:.1f}fps):'
        )

        for result in results:
            boxes = result.boxes

            if len(boxes) == 0:
                self.get_logger().info('No objects detected')
            else:
                self.get_logger().info(f'Detected {len(boxes)} object(s):')

                for i, box in enumerate(boxes):
                    cls_id = int(box.cls[0])
                    class_name = self._resolve_class_name(result.names, cls_id)
                    confidence = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                    self.get_logger().info(
                        f'  [{i + 1}] {class_name} (ID:{cls_id}): {confidence:.2f} '
                        f'(x1:{int(x1)}, y1:{int(y1)}, x2:{int(x2)}, y2:{int(y2)})'
                    )

        self.get_logger().info('=' * 60)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = YoloNCNNDetectorNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down yolo_ncnn_detector_node.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
