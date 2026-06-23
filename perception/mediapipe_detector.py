"""
MediaPipe-based object detection and 3D position estimation.
Runs on Raspberry Pi — targets <100ms inference latency.

Pipeline:
  RGB frame → MediaPipe detection → bounding box centroid
  → depth lookup → pixel-to-3D → robot base frame transform
"""

import cv2
import numpy as np
import mediapipe as mp
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CameraIntrinsics:
    fx: float = 615.0   # focal length x (pixels)
    fy: float = 615.0   # focal length y (pixels)
    cx: float = 320.0   # principal point x
    cy: float = 240.0   # principal point y
    width: int = 640
    height: int = 480


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]   # x1, y1, x2, y2 (pixels)
    centroid_px: Tuple[int, int]       # u, v (pixels)
    position_cam: Optional[np.ndarray] = None   # [x, y, z] in camera frame (m)
    position_base: Optional[np.ndarray] = None  # [x, y, z] in robot base frame (m)


# Extrinsic: camera → robot base frame
# Adjust translation and rotation to match your mount geometry
CAM_TO_BASE_R = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0],
], dtype=float)

CAM_TO_BASE_T = np.array([0.12, 0.0, 0.18], dtype=float)  # meters


class MediaPipeDetector:
    """
    Wraps MediaPipe object detection for on-device use on RPi.
    Uses the EfficientDet-Lite0 model (fast, embedded-friendly).
    """

    def __init__(
        self,
        model_path: str = "efficientdet_lite0.tflite",
        score_threshold: float = 0.5,
        intrinsics: CameraIntrinsics = None,
    ):
        self.intrinsics = intrinsics or CameraIntrinsics()
        self.score_threshold = score_threshold

        # MediaPipe object detector setup
        BaseOptions = mp.tasks.BaseOptions
        ObjectDetector = mp.tasks.vision.ObjectDetector
        ObjectDetectorOptions = mp.tasks.vision.ObjectDetectorOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        options = ObjectDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            max_results=5,
            score_threshold=score_threshold,
            running_mode=VisionRunningMode.IMAGE,
        )
        self.detector = ObjectDetector.create_from_options(options)

    def detect(
        self,
        frame: np.ndarray,
        depth_frame: Optional[np.ndarray] = None,
    ) -> list[Detection]:
        """
        Run detection on an RGB frame.

        Args:
            frame: HxWx3 uint8 RGB image
            depth_frame: HxW float32 depth in meters (optional)

        Returns:
            List of Detection objects
        """
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        result = self.detector.detect(mp_image)

        detections = []
        h, w = frame.shape[:2]

        for det in result.detections:
            bbox = det.bounding_box
            x1 = int(bbox.origin_x)
            y1 = int(bbox.origin_y)
            x2 = int(bbox.origin_x + bbox.width)
            y2 = int(bbox.origin_y + bbox.height)

            u = (x1 + x2) // 2
            v = (y1 + y2) // 2

            label = det.categories[0].category_name
            conf = det.categories[0].score

            d = Detection(
                label=label,
                confidence=conf,
                bbox=(x1, y1, x2, y2),
                centroid_px=(u, v),
            )

            if depth_frame is not None:
                d.position_cam = self._pixel_to_camera(u, v, depth_frame)
                if d.position_cam is not None:
                    d.position_base = self._camera_to_base(d.position_cam)

            detections.append(d)

        return detections

    def _pixel_to_camera(
        self, u: int, v: int, depth: np.ndarray
    ) -> Optional[np.ndarray]:
        """Back-project pixel (u, v) to 3D point in camera frame."""
        k = self.intrinsics
        z = float(depth[v, u])
        if z <= 0.01 or z > 2.0:
            return None
        x = (u - k.cx) * z / k.fx
        y = (v - k.cy) * z / k.fy
        return np.array([x, y, z])

    def _camera_to_base(self, p_cam: np.ndarray) -> np.ndarray:
        """Transform point from camera frame to robot base frame."""
        return CAM_TO_BASE_R @ p_cam + CAM_TO_BASE_T
