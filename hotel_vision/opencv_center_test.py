"""수직 카메라 화면에서 캔 윗면 원과 ROI 중심을 확인하는 ROS 2 노드."""
# opencv_center_test
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"

WINDOW_NAME = "OpenCV Can Center Test"
ROTATE_CAMERA_180 = False

# 사진을 기준으로 한 첫 ROI 설정값입니다.
# S 키로 기록한 u/width, v/height 값을 여기에 복사하여 보정합니다.
ROI_CENTER_X_RATIO = 0.6508
ROI_CENTER_Y_RATIO = 0.8236
ROI_WIDTH_RATIO = 0.32
ROI_HEIGHT_RATIO = 0.40
ROI_BOTTOM_LIMIT_RATIO = 0.99

# Hough Circle 기본값입니다. 실제 화면에서 원이 누락되면 PARAM2를 낮추고,
# 오검출이 많으면 PARAM2를 높입니다.
HOUGH_DP = 1.2
HOUGH_PARAM1 = 100.0
HOUGH_PARAM2 = 28.0
HOUGH_MIN_RADIUS_RATIO = 0.035
HOUGH_MAX_RADIUS_RATIO = 0.13
HOUGH_MIN_DISTANCE_RATIO = 0.10

CAN_DIAMETER_MM = 66.0
MIN_CAN_DIAMETER_MM = 45.0
MAX_CAN_DIAMETER_MM = 85.0
MIN_DEPTH_MM = 100.0
MAX_DEPTH_MM = 2000.0
MIN_VALID_DEPTH_PIXELS = 30
MIN_VALID_DEPTH_RATIO = 0.10
RGB_DEPTH_MAX_STAMP_DIFF_SEC = 0.08

RECORD_SAMPLE_COUNT = 20
RECORD_CENTER_MAX_SPREAD_PX = 8.0
RECORD_DEPTH_MAX_RANGE_MM = 20.0


@dataclass(frozen=True)
class CircleObservation:
    """전체 영상 좌표로 표현된 원 검출 결과."""

    x: int
    y: int
    radius: int
    depth_mm: Optional[float] = None
    diameter_mm: Optional[float] = None


def message_stamp_sec(msg) -> float:
    """ROS 메시지 Header stamp를 초 단위로 반환한다."""
    return float(msg.header.stamp.sec) + float(
        msg.header.stamp.nanosec
    ) * 1.0e-9


def calculate_roi_bounds(
    width: int,
    height: int,
) -> Tuple[int, int, int, int, int, int]:
    """비율 설정을 실제 ROI 경계와 기준점 픽셀로 변환한다."""
    target_x = int(round(width * ROI_CENTER_X_RATIO))
    target_y = int(round(height * ROI_CENTER_Y_RATIO))
    roi_width = max(1, int(round(width * ROI_WIDTH_RATIO)))
    roi_height = max(1, int(round(height * ROI_HEIGHT_RATIO)))

    x1 = max(0, target_x - roi_width // 2)
    x2 = min(width, target_x + roi_width // 2)
    y1 = max(0, target_y - roi_height // 2)
    y2 = min(
        height,
        target_y + roi_height // 2,
        int(round(height * ROI_BOTTOM_LIMIT_RATIO)),
    )

    if x2 <= x1 or y2 <= y1:
        raise ValueError(
            f"ROI 설정이 잘못되었습니다: {(x1, y1, x2, y2)}"
        )

    return x1, y1, x2, y2, target_x, target_y


def stable_circle_depth_mm(
    depth_frame: Optional[np.ndarray],
    circle_x: int,
    circle_y: int,
    circle_radius: int,
) -> Optional[float]:
    """캔 윗면 원 내부의 유효 Depth 중앙값을 반환한다."""
    if depth_frame is None:
        return None

    height, width = depth_frame.shape[:2]
    sample_radius = max(4, int(round(circle_radius * 0.50)))

    x1 = max(0, circle_x - sample_radius)
    x2 = min(width, circle_x + sample_radius + 1)
    y1 = max(0, circle_y - sample_radius)
    y2 = min(height, circle_y + sample_radius + 1)

    if x2 <= x1 or y2 <= y1:
        return None

    depth_roi = np.asarray(
        depth_frame[y1:y2, x1:x2],
        dtype=np.float64,
    )
    yy, xx = np.ogrid[y1:y2, x1:x2]
    disk_mask = (
        (xx - circle_x) ** 2 + (yy - circle_y) ** 2
        <= sample_radius ** 2
    )
    valid_mask = (
        disk_mask
        & np.isfinite(depth_roi)
        & (depth_roi >= MIN_DEPTH_MM)
        & (depth_roi <= MAX_DEPTH_MM)
    )
    valid_depths = depth_roi[valid_mask]
    disk_pixel_count = int(np.count_nonzero(disk_mask))

    if (
        valid_depths.size < MIN_VALID_DEPTH_PIXELS
        or valid_depths.size
        < disk_pixel_count * MIN_VALID_DEPTH_RATIO
    ):
        return None

    median_depth = float(np.median(valid_depths))
    absolute_deviation = np.abs(valid_depths - median_depth)
    mad = float(np.median(absolute_deviation))
    threshold = max(5.0, 3.0 * mad)
    filtered_depths = valid_depths[
        absolute_deviation <= threshold
    ]

    if filtered_depths.size < MIN_VALID_DEPTH_PIXELS:
        return None

    return float(np.median(filtered_depths))


def estimate_circle_diameter_mm(
    radius_px: int,
    depth_mm: Optional[float],
    intrinsics: Optional[Dict[str, float]],
) -> Optional[float]:
    """픽셀 반지름과 Depth로 실제 원 지름을 근사한다."""
    if depth_mm is None or intrinsics is None:
        return None

    focal_length = (
        float(intrinsics["fx"]) + float(intrinsics["fy"])
    ) / 2.0
    if focal_length <= 0.0:
        return None

    return float(2.0 * radius_px * depth_mm / focal_length)


def detect_can_top_circle(
    color_frame: np.ndarray,
    depth_frame: Optional[np.ndarray] = None,
    intrinsics: Optional[Dict[str, float]] = None,
) -> Tuple[
    Optional[CircleObservation],
    List[CircleObservation],
    Tuple[int, int, int, int, int, int],
]:
    """ROI에서 캔 윗면 후보를 찾고 기준점에 가장 가까운 원을 선택한다."""
    height, width = color_frame.shape[:2]
    roi_info = calculate_roi_bounds(width, height)
    x1, y1, x2, y2, target_x, target_y = roi_info
    roi = color_frame[y1:y2, x1:x2]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(
        clipLimit=2.0,
        tileGridSize=(8, 8),
    )
    gray = clahe.apply(gray)
    gray = cv2.medianBlur(gray, 7)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=max(1, int(round(width * HOUGH_MIN_DISTANCE_RATIO))),
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=max(
            1,
            int(round(width * HOUGH_MIN_RADIUS_RATIO)),
        ),
        maxRadius=max(
            2,
            int(round(width * HOUGH_MAX_RADIUS_RATIO)),
        ),
    )

    observations: List[CircleObservation] = []
    scored_candidates = []

    if circles is not None:
        rounded = np.rint(circles[0]).astype(int)

        for local_x, local_y, radius in rounded:
            global_x = int(x1 + local_x)
            global_y = int(y1 + local_y)
            radius = int(radius)

            # 원 전체가 ROI 안에 들어온 후보만 사용한다.
            if (
                global_x - radius < x1
                or global_x + radius >= x2
                or global_y - radius < y1
                or global_y + radius >= y2
            ):
                continue

            depth_mm = stable_circle_depth_mm(
                depth_frame,
                global_x,
                global_y,
                radius,
            )
            diameter_mm = estimate_circle_diameter_mm(
                radius,
                depth_mm,
                intrinsics,
            )
            observation = CircleObservation(
                x=global_x,
                y=global_y,
                radius=radius,
                depth_mm=depth_mm,
                diameter_mm=diameter_mm,
            )
            observations.append(observation)

            if (
                diameter_mm is not None
                and not (
                    MIN_CAN_DIAMETER_MM
                    <= diameter_mm
                    <= MAX_CAN_DIAMETER_MM
                )
            ):
                continue

            center_distance = float(
                np.hypot(
                    global_x - target_x,
                    global_y - target_y,
                )
            )
            diameter_penalty = (
                0.0
                if diameter_mm is None
                else abs(diameter_mm - CAN_DIAMETER_MM) * 2.0
            )
            scored_candidates.append(
                (
                    center_distance + diameter_penalty,
                    observation,
                )
            )

    selected = None
    if scored_candidates:
        selected = min(
            scored_candidates,
            key=lambda item: item[0],
        )[1]

    return selected, observations, roi_info


def draw_detection_overlay(
    color_frame: np.ndarray,
    selected: Optional[CircleObservation],
    candidates: List[CircleObservation],
    roi_info: Tuple[int, int, int, int, int, int],
) -> np.ndarray:
    """ROI, 기준점, 후보 원과 선택 원을 영상에 표시한다."""
    output = color_frame.copy()
    height, width = output.shape[:2]
    x1, y1, x2, y2, target_x, target_y = roi_info

    cv2.rectangle(output, (x1, y1), (x2, y2), (255, 0, 255), 2)
    cv2.drawMarker(
        output,
        (width // 2, height // 2),
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=18,
        thickness=1,
    )
    cv2.drawMarker(
        output,
        (target_x, target_y),
        (0, 0, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=2,
    )

    for candidate in candidates:
        cv2.circle(
            output,
            (candidate.x, candidate.y),
            candidate.radius,
            (0, 255, 255),
            2,
        )

    if selected is not None:
        cv2.circle(
            output,
            (selected.x, selected.y),
            selected.radius,
            (0, 255, 0),
            3,
        )
        cv2.circle(
            output,
            (selected.x, selected.y),
            5,
            (0, 0, 255),
            -1,
        )
        depth_text = (
            "depth=N/A"
            if selected.depth_mm is None
            else f"depth={selected.depth_mm:.1f}mm"
        )
        diameter_text = (
            "diameter=N/A"
            if selected.diameter_mm is None
            else f"diameter={selected.diameter_mm:.1f}mm"
        )
        cv2.putText(
            output,
            (
                f"selected=({selected.x},{selected.y}) "
                f"r={selected.radius} {depth_text} {diameter_text}"
            ),
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )

    cv2.putText(
        output,
        "S: record 20 frames | click: record pixel | ESC/Q: exit",
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    return output


class OpenCvCenterTestNode(Node):
    """로봇을 움직이지 않고 ROI와 캔 원 중심을 기록한다."""

    def __init__(self) -> None:
        super().__init__("opencv_center_test")

        self.bridge = CvBridge()
        self.color_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.color_stamp: Optional[float] = None
        self.depth_stamp: Optional[float] = None
        self.intrinsics: Optional[Dict[str, float]] = None
        self.frame_generation = 0
        self.last_processed_generation = -1
        self.latest_selected: Optional[CircleObservation] = None
        self.latest_frame_shape: Optional[Tuple[int, int]] = None
        self.recording = False
        self.record_samples: List[Tuple[float, float, float]] = []
        self.window_created = False

        self.color_subscription = self.create_subscription(
            Image,
            COLOR_TOPIC,
            self.color_callback,
            qos_profile_sensor_data,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.process_frame)

        self.get_logger().info(
            "ROI 원 중심 테스트를 시작합니다. 로봇은 움직이지 않습니다."
        )
        self.get_logger().info(
            "캔을 실제 파지 중심 아래에 놓고, 원과 Depth 표시를 확인한 뒤 "
            "S를 누르세요."
        )

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
            self.color_frame = np.asarray(frame)
            self.color_stamp = message_stamp_sec(msg)
            self.frame_generation += 1
        except Exception as error:
            self.get_logger().error(f"RGB 변환 실패: {error}")

    def depth_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            if frame.dtype == np.float32:
                frame = frame * 1000.0
            self.depth_frame = np.asarray(frame)
            self.depth_stamp = message_stamp_sec(msg)
        except Exception as error:
            self.get_logger().error(f"Depth 변환 실패: {error}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if float(msg.k[0]) <= 0.0 or float(msg.k[4]) <= 0.0:
            self.get_logger().warning("유효하지 않은 CameraInfo를 무시합니다.")
            return

        ppx = float(msg.k[2])
        ppy = float(msg.k[5])
        if ROTATE_CAMERA_180:
            ppx = float(msg.width - 1) - ppx
            ppy = float(msg.height - 1) - ppy

        self.intrinsics = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "ppx": ppx,
            "ppy": ppy,
        }

    def process_frame(self) -> None:
        if (
            self.color_frame is None
            or self.frame_generation == self.last_processed_generation
        ):
            return

        self.last_processed_generation = self.frame_generation
        color_frame = self.color_frame.copy()
        depth_frame = (
            None
            if self.depth_frame is None
            else self.depth_frame.copy()
        )

        if (
            depth_frame is not None
            and color_frame.shape[:2] != depth_frame.shape[:2]
        ):
            depth_frame = None

        if (
            self.color_stamp is not None
            and self.depth_stamp is not None
            and abs(self.color_stamp - self.depth_stamp)
            > RGB_DEPTH_MAX_STAMP_DIFF_SEC
        ):
            depth_frame = None

        if ROTATE_CAMERA_180:
            color_frame = cv2.rotate(
                color_frame,
                cv2.ROTATE_180,
            )
            if depth_frame is not None:
                depth_frame = cv2.rotate(
                    depth_frame,
                    cv2.ROTATE_180,
                )

        selected, candidates, roi_info = detect_can_top_circle(
            color_frame,
            depth_frame,
            self.intrinsics,
        )
        self.latest_selected = selected
        self.latest_frame_shape = color_frame.shape[:2]

        if (
            self.recording
            and selected is not None
            and selected.depth_mm is not None
        ):
            depth_value = selected.depth_mm
            self.record_samples.append(
                (
                    float(selected.x),
                    float(selected.y),
                    depth_value,
                )
            )

            if len(self.record_samples) >= RECORD_SAMPLE_COUNT:
                self.finish_recording()

        output = draw_detection_overlay(
            color_frame,
            selected,
            candidates,
            roi_info,
        )

        try:
            if not self.window_created:
                # 마우스 좌표가 원본 이미지 픽셀과 같도록 자동 크기를 사용한다.
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
                cv2.setMouseCallback(
                    WINDOW_NAME,
                    self.mouse_callback,
                )
                self.window_created = True

            cv2.imshow(WINDOW_NAME, output)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            self.get_logger().error(f"OpenCV 창 표시 실패: {error}")
            rclpy.shutdown()
            return

        if key in (ord("q"), 27):
            rclpy.shutdown()
        elif key == ord("s"):
            self.recording = True
            self.record_samples.clear()
            self.get_logger().info(
                f"새 원 중심 {RECORD_SAMPLE_COUNT}개 기록을 시작합니다."
            )

    def finish_recording(self) -> None:
        samples = np.asarray(
            self.record_samples,
            dtype=np.float64,
        )
        median_x = float(np.median(samples[:, 0]))
        median_y = float(np.median(samples[:, 1]))
        center_spread = float(
            np.max(
                np.linalg.norm(
                    samples[:, :2] - [median_x, median_y],
                    axis=1,
                )
            )
        )
        valid_depths = samples[np.isfinite(samples[:, 2]), 2]
        median_depth = (
            None
            if valid_depths.size == 0
            else float(np.median(valid_depths))
        )

        depth_range = (
            float("inf")
            if valid_depths.size == 0
            else float(np.ptp(valid_depths))
        )
        if (
            center_spread > RECORD_CENTER_MAX_SPREAD_PX
            or depth_range > RECORD_DEPTH_MAX_RANGE_MM
        ):
            self.get_logger().warning(
                "서로 다른 원 또는 흔들리는 원이 섞여 기록을 폐기합니다: "
                f"center_spread={center_spread:.1f}px, "
                f"depth_range={depth_range:.1f}mm. S를 눌러 다시 기록하세요."
            )
            self.recording = False
            self.record_samples.clear()
            return

        if self.latest_frame_shape is None:
            return

        height, width = self.latest_frame_shape
        self.get_logger().info(
            "원 중심 기록 완료: "
            f"pixel=({median_x:.1f}, {median_y:.1f}), "
            f"ratio=({median_x / width:.4f}, "
            f"{median_y / height:.4f}), "
            f"depth={median_depth if median_depth is not None else 'N/A'}"
        )
        self.get_logger().info(
            "위 ratio 값을 ROI_CENTER_X_RATIO와 "
            "ROI_CENTER_Y_RATIO에 입력하세요."
        )
        self.recording = False
        self.record_samples.clear()

    def mouse_callback(
        self,
        event: int,
        x: int,
        y: int,
        _flags: int,
        _param,
    ) -> None:
        if (
            event != cv2.EVENT_LBUTTONDOWN
            or self.latest_frame_shape is None
        ):
            return

        height, width = self.latest_frame_shape
        self.get_logger().info(
            f"클릭 픽셀=({x}, {y}), "
            f"ratio=({x / width:.4f}, {y / height:.4f})"
        )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenCvCenterTestNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
