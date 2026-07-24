#!/usr/bin/env python3
"""2D 웹캠에서 손의 위치와 주먹 여부를 검출해 ROS 2 토픽으로 발행합니다.

이 노드는 영상 인식만 담당합니다.
Doosan 로봇을 실제로 움직이는 코드는 이후에 만들
``hand_follow_robot_node``에 분리할 예정입니다.

발행 토픽
----------
* /hand_teleop/hand_position (geometry_msgs/PointStamped)
    - point.x: 화면 안 손 중심의 정규화 X 좌표, 왼쪽 0.0 / 오른쪽 1.0
    - point.y: 화면 안 손 중심의 정규화 Y 좌표, 위쪽 0.0 / 아래쪽 1.0
    - point.z: 현재는 사용하지 않으므로 0.0
* /hand_teleop/hand_detected (std_msgs/Bool)
    - 현재 프레임에서 손을 찾았는지 여부
* /hand_teleop/fist (std_msgs/Bool)
    - 여러 프레임 동안 확인된 안정화된 주먹 여부

중요
----
이 노드가 발행하는 X/Y는 로봇의 mm 좌표가 아니라 0.0~1.0 영상 좌표입니다.
다음 단계의 로봇 제어 노드에서 아래와 같이 변환할 예정입니다.

    손 화면 +X -> 로봇 BASE +Y
    손 화면 +Y -> 로봇 BASE -Z

MediaPipe Tasks API는 별도의 ``hand_landmarker.task`` 모델 파일을 필요로 합니다.
모델은 패키지의 ``models/hand_landmarker.task``에 두거나,
실행 시 ``model_path`` ROS 파라미터로 절대경로를 전달할 수 있습니다.
"""

# export PYTHONNOUSERSITE=1 하기 실행할 때
# hand_tracker_node

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Optional, Sequence, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool


# Google에서 제공하는 Hand Landmarker 모델의 공식 저장 위치입니다.
# 노드가 인터넷에서 파일을 자동으로 받지는 않습니다.
# 실행 환경을 재현하기 쉽도록 사용자가 한 번 내려받아 패키지에 보관합니다.
MODEL_DOWNLOAD_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
)


# MediaPipe 손 랜드마크 번호 연결 관계입니다.
# 디버그 영상에 손가락 뼈대를 그릴 때 사용합니다.
HAND_CONNECTIONS = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),      # 엄지
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),      # 검지
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),    # 중지
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),    # 약지
    (13, 17),
    (17, 18),
    (18, 19),
    (19, 20),    # 소지
    (0, 17),
)


# 손바닥 중심 계산에 사용하는 랜드마크입니다.
# 0: 손목, 5/9/13/17: 네 손가락의 MCP 관절(손바닥과 손가락 경계)
PALM_CENTER_LANDMARKS = (0, 5, 9, 13, 17)


# 네 손가락의 PIP 관절과 손끝 랜드마크 쌍입니다.
# 주먹 여부를 판단할 때 손끝이 PIP보다 손바닥 중심에 가까운지 확인합니다.
# 엄지는 손의 좌우 방향에 따라 모양 변화가 커서 초안의 주먹 판정에서는 제외합니다.
FINGER_PIP_TIP_PAIRS = (
    (6, 8),     # 검지
    (10, 12),   # 중지
    (14, 16),   # 약지
    (18, 20),   # 소지
)


NormalizedPoint = Tuple[float, float]


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """value를 minimum~maximum 범위 안으로 제한합니다."""

    return max(minimum, min(maximum, value))


def calculate_palm_center(landmarks: Sequence[Any]) -> NormalizedPoint:
    """손목과 네 MCP 관절의 평균으로 안정적인 손바닥 중심을 계산합니다.

    손가락 끝 하나를 기준으로 삼으면 손가락을 펴거나 주먹을 쥘 때 좌표가 크게
    변합니다. 손바닥을 구성하는 여러 점의 평균을 사용하면 이런 영향을 줄일 수
    있습니다.
    """

    x_sum = sum(float(landmarks[index].x) for index in PALM_CENTER_LANDMARKS)
    y_sum = sum(float(landmarks[index].y) for index in PALM_CENTER_LANDMARKS)
    count = float(len(PALM_CENTER_LANDMARKS))

    # 손이 화면 가장자리에 걸리면 MediaPipe 좌표가 아주 조금 0~1을 벗어날 수
    # 있으므로 발행 전 안전하게 범위를 제한합니다.
    return (
        _clamp(x_sum / count, 0.0, 1.0),
        _clamp(y_sum / count, 0.0, 1.0),
    )


def estimate_fist(
    landmarks: Sequence[Any],
    tip_distance_ratio: float,
    minimum_folded_fingers: int,
) -> Tuple[bool, int]:
    """랜드마크 모양으로 현재 손이 주먹인지 간단히 추정합니다.

    각 손가락에 대해 다음 두 거리를 비교합니다.

    * 손바닥 중심에서 PIP 관절까지의 거리
    * 손바닥 중심에서 손끝까지의 거리

    손가락이 펴져 있으면 보통 손끝이 PIP보다 손바닥 중심에서 멉니다.
    손가락이 접히면 손끝이 손바닥 쪽으로 들어오므로 두 거리가 비슷해지거나
    손끝 거리가 더 짧아집니다.

    이것은 학습된 제스처 분류기가 아니라 이해하기 쉬운 초안용 규칙입니다.
    카메라 각도와 사용자의 손 모양에 맞춰 ``fist_tip_distance_ratio``와
    ``fist_min_folded_fingers``를 나중에 조정할 수 있습니다.
    """

    center_x, center_y = calculate_palm_center(landmarks)

    # 작은 임시 좌표 객체를 만드는 대신 좌표 차이를 직접 계산합니다.
    folded_count = 0

    for pip_index, tip_index in FINGER_PIP_TIP_PAIRS:
        pip = landmarks[pip_index]
        tip = landmarks[tip_index]

        pip_distance = math.hypot(float(pip.x) - center_x, float(pip.y) - center_y)
        tip_distance = math.hypot(float(tip.x) - center_x, float(tip.y) - center_y)

        # ratio가 커질수록 손가락을 "접혔다"고 판단하기 쉬워집니다.
        if tip_distance <= pip_distance * tip_distance_ratio:
            folded_count += 1

    return folded_count >= minimum_folded_fingers, folded_count


class HandTrackerNode(Node):
    """웹캠 영상을 처리하고 손 상태 토픽을 발행하는 ROS 2 노드입니다."""

    def __init__(self, node_name: str = "hand_tracker_node") -> None:
        # 깊이 추정 버전이 카메라·MediaPipe·필터 구현을 그대로 재사용할 수
        # 있도록 노드 이름만 선택적으로 받을 수 있게 합니다. 기본값은 기존과
        # 같으므로 hand_tracker_node의 동작에는 변화가 없습니다.
        super().__init__(node_name)
        self._position_frame_id = "webcam_normalized"

        # ------------------------------------------------------------------
        # ROS 파라미터 선언
        # ------------------------------------------------------------------
        # ROS 2 파라미터는 일반적으로 None을 기본값으로 선언하기 어렵습니다.
        # 이 영상 노드가 실제로 실행되려면 카메라 번호, FPS 같은 값도 반드시
        # 필요하므로 안전한 시작값을 제공합니다. 모든 값은 실행 명령이나 YAML로
        # 바꿀 수 있습니다.

        # MediaPipe 모델 경로입니다.
        # 빈 문자열이면 패키지 안 models/hand_landmarker.task를 자동 탐색합니다.
        self.declare_parameter("model_path", "")

        # /dev/videoN의 N에 해당하는 OpenCV 카메라 번호입니다.
        # RealSense와 일반 웹캠이 함께 연결되면 0이 아닐 수 있습니다.
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("camera_fps", 30.0)

        # true이면 사람이 거울을 보는 것처럼 영상을 좌우 반전합니다.
        # 반전된 영상을 MediaPipe에도 입력하므로 화면 오른쪽 이동은 +X가 됩니다.
        self.declare_parameter("mirror_image", True)

        # OpenCV 디버그 창 표시 여부입니다. GUI가 없는 환경에서는 false로 합니다.
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_name", "Hand Teleop Tracker")

        # 영상 처리 타이머 주기입니다. 카메라 FPS와 같거나 조금 낮게 설정합니다.
        self.declare_parameter("processing_rate_hz", 30.0)

        # MediaPipe가 손을 채택하기 위한 신뢰도 기준입니다.
        self.declare_parameter("min_hand_detection_confidence", 0.5)
        self.declare_parameter("min_hand_presence_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        # 손 중심 좌표 저역통과 필터 계수입니다.
        # 1.0이면 필터 없이 새 좌표를 그대로 사용합니다.
        # 값이 작을수록 부드럽지만 손 움직임에 대한 반응이 느려집니다.
        self.declare_parameter("filter_alpha", 0.25)

        # 주먹 판정용 초안 수치입니다.
        # ratio가 클수록 손가락을 접힌 것으로 판단하기 쉬워집니다.
        self.declare_parameter("fist_tip_distance_ratio", 1.15)
        self.declare_parameter("fist_min_folded_fingers", 3)

        # 한 프레임의 오검출로 로봇 동작이 바뀌지 않도록 연속 확인합니다.
        self.declare_parameter("fist_confirm_frames", 8)
        self.declare_parameter("fist_release_frames", 3)

        # 토픽 이름도 파라미터화해 다른 시스템과 통합할 때 쉽게 변경합니다.
        self.declare_parameter("position_topic", "/hand_teleop/hand_position")
        self.declare_parameter("detected_topic", "/hand_teleop/hand_detected")
        self.declare_parameter("fist_topic", "/hand_teleop/fist")

        # ------------------------------------------------------------------
        # 파라미터 읽기 및 유효성 검사
        # ------------------------------------------------------------------
        self._camera_index = int(self.get_parameter("camera_index").value)
        self._frame_width = int(self.get_parameter("frame_width").value)
        self._frame_height = int(self.get_parameter("frame_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._mirror_image = bool(self.get_parameter("mirror_image").value)
        self._show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self._window_name = str(self.get_parameter("window_name").value)
        self._processing_rate_hz = float(
            self.get_parameter("processing_rate_hz").value
        )

        self._filter_alpha = float(self.get_parameter("filter_alpha").value)
        self._fist_tip_distance_ratio = float(
            self.get_parameter("fist_tip_distance_ratio").value
        )
        self._fist_min_folded_fingers = int(
            self.get_parameter("fist_min_folded_fingers").value
        )
        self._fist_confirm_frames = int(
            self.get_parameter("fist_confirm_frames").value
        )
        self._fist_release_frames = int(
            self.get_parameter("fist_release_frames").value
        )

        if self._processing_rate_hz <= 0.0:
            raise ValueError("processing_rate_hz는 0보다 커야 합니다.")
        if not 0.0 < self._filter_alpha <= 1.0:
            raise ValueError("filter_alpha는 0보다 크고 1 이하여야 합니다.")
        if self._fist_tip_distance_ratio <= 0.0:
            raise ValueError("fist_tip_distance_ratio는 0보다 커야 합니다.")
        if not 1 <= self._fist_min_folded_fingers <= 4:
            raise ValueError("fist_min_folded_fingers는 1~4 범위여야 합니다.")
        if self._fist_confirm_frames < 1 or self._fist_release_frames < 1:
            raise ValueError("주먹 확인 프레임 수는 1 이상이어야 합니다.")

        # ------------------------------------------------------------------
        # ROS Publisher 생성@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        # ------------------------------------------------------------------
        position_topic = str(self.get_parameter("position_topic").value)
        detected_topic = str(self.get_parameter("detected_topic").value)
        fist_topic = str(self.get_parameter("fist_topic").value)

        self._position_publisher = self.create_publisher(
            PointStamped, position_topic, 10
        )
        self._detected_publisher = self.create_publisher(
            Bool, detected_topic, 10
        )
        self._fist_publisher = self.create_publisher(Bool, fist_topic, 10)

        # ------------------------------------------------------------------
        # MediaPipe Hand Landmarker 초기화
        # ------------------------------------------------------------------
        configured_model_path = str(self.get_parameter("model_path").value)
        self._model_path = self._resolve_model_path(configured_model_path)

        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(
                model_asset_path=str(self._model_path)
            ),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=float(
                self.get_parameter("min_hand_detection_confidence").value
            ),
            min_hand_presence_confidence=float(
                self.get_parameter("min_hand_presence_confidence").value
            ),
            min_tracking_confidence=float(
                self.get_parameter("min_tracking_confidence").value
            ),
        )
        self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(
            options
        )

        # VIDEO 모드의 timestamp는 프레임마다 반드시 증가해야 합니다.
        self._last_mediapipe_timestamp_ms = -1

        # ------------------------------------------------------------------
        # 웹캠 초기화
        # ------------------------------------------------------------------
        self._capture = self._open_camera()

        # ------------------------------------------------------------------
        # 프레임 간 상태
        # ------------------------------------------------------------------
        # None은 현재까지 유효한 손 중심이 없다는 뜻입니다.
        # 로봇 좌표의 미정값이 아니라 영상 필터의 내부 상태로만 사용합니다.
        self._filtered_center: Optional[NormalizedPoint] = None

        # 주먹을 쥐고/펴는 상태를 여러 프레임 확인하기 위한 카운터입니다.
        self._fist_on_count = 0
        self._fist_off_count = 0
        self._stable_fist = False

        # 카메라 읽기 오류가 계속될 때 로그가 화면을 가득 채우지 않도록 사용합니다.
        self._last_camera_error_log_time = 0.0

        # destroy_node()가 여러 경로에서 불리더라도 장치를 한 번만 닫습니다.
        self._cleaned_up = False

        self._timer = self.create_timer(
            1.0 / self._processing_rate_hz,
            self._process_frame,
        )

        actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self._capture.get(cv2.CAP_PROP_FPS))

        self.get_logger().info(
            "손 추적을 시작합니다. "
            f"camera_index={self._camera_index}, "
            f"capture={actual_width}x{actual_height}@{actual_fps:.1f}fps"
        )
        self.get_logger().info(f"MediaPipe 모델: {self._model_path}")
        self.get_logger().info(
            "좌표 규칙: 화면 오른쪽=+X, 화면 아래=+Y, 발행 범위=0.0~1.0"
        )
        self.get_logger().info(
            f"토픽: position={position_topic}, "
            f"detected={detected_topic}, fist={fist_topic}"
        )
        if self._show_debug_window:
            self.get_logger().info("디버그 창 종료: Q 또는 ESC")

    def _resolve_model_path(self, configured_path: str) -> Path:
        """ROS 파라미터 또는 패키지 기본 위치에서 모델 파일을 찾습니다."""

        candidates = []

        if configured_path.strip():
            candidate = Path(configured_path).expanduser()
            if not candidate.is_absolute():
                candidate = Path.cwd() / candidate
            candidates.append(candidate)

        # 설치된 패키지의 share/hand_teleop/models 경로를 먼저 확인합니다.
        try:
            share_directory = Path(get_package_share_directory("hand_teleop"))
            candidates.append(
                share_directory / "models" / "hand_landmarker.task"
            )
        except PackageNotFoundError:
            # 아직 한 번도 빌드하지 않은 소스 직접 실행 상황일 수 있습니다.
            pass

        # --symlink-install 또는 소스 직접 실행 시 사용할 경로입니다.
        source_package_root = Path(__file__).resolve().parents[1]
        candidates.append(
            source_package_root / "models" / "hand_landmarker.task"
        )

        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()

        checked_paths = "\n".join(f"  - {path}" for path in candidates)
        raise FileNotFoundError(
            "MediaPipe hand_landmarker.task 모델을 찾을 수 없습니다.\n"
            f"확인한 경로:\n{checked_paths}\n\n"
            "다음 위치에 모델을 내려받으세요:\n"
            "  ~/cobot_ws/src/hand_teleop/models/hand_landmarker.task\n"
            f"다운로드 URL:\n  {MODEL_DOWNLOAD_URL}\n\n"
            "또는 실행할 때 다음처럼 경로를 지정할 수 있습니다:\n"
            "  --ros-args -p model_path:=/절대경로/hand_landmarker.task"
        )

    def _open_camera(self) -> cv2.VideoCapture:
        """설정한 번호의 웹캠을 열고 해상도와 FPS를 적용합니다."""

        # Ubuntu의 일반 USB 웹캠에는 V4L2 백엔드가 가장 명확합니다.
        capture = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)

        # 일부 카메라/가상 카메라는 명시적인 V4L2 지정에 실패할 수 있어
        # OpenCV 기본 백엔드로 한 번 더 시도합니다.
        if not capture.isOpened():
            capture.release()
            capture = cv2.VideoCapture(self._camera_index)

        if not capture.isOpened():
            raise RuntimeError(
                f"camera_index={self._camera_index} 웹캠을 열 수 없습니다. "
                "v4l2-ctl --list-devices 또는 /dev/video*를 확인하세요."
            )

        if self._frame_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
        if self._frame_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
        if self._camera_fps > 0.0:
            capture.set(cv2.CAP_PROP_FPS, self._camera_fps)

        # 가능한 카메라에서는 오래된 프레임이 큐에 쌓이지 않도록 버퍼를 줄입니다.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _next_mediapipe_timestamp_ms(self) -> int:
        """MediaPipe VIDEO 모드용 단조 증가 timestamp를 만듭니다."""

        timestamp_ms = int(time.monotonic() * 1000.0)
        if timestamp_ms <= self._last_mediapipe_timestamp_ms:
            timestamp_ms = self._last_mediapipe_timestamp_ms + 1

        self._last_mediapipe_timestamp_ms = timestamp_ms
        return timestamp_ms

    def _low_pass_filter(self, raw_center: NormalizedPoint) -> NormalizedPoint:
        """손 중심의 작은 떨림을 지수 이동 평균으로 줄입니다."""

        if self._filtered_center is None:
            self._filtered_center = raw_center
            return raw_center

        old_x, old_y = self._filtered_center
        raw_x, raw_y = raw_center
        alpha = self._filter_alpha

        filtered = (
            alpha * raw_x + (1.0 - alpha) * old_x,
            alpha * raw_y + (1.0 - alpha) * old_y,
        )
        self._filtered_center = filtered
        return filtered

    def _update_stable_fist(self, raw_fist: bool) -> bool:
        """연속 프레임 확인으로 주먹 상태의 순간적인 오검출을 제거합니다."""

        if raw_fist:
            self._fist_on_count += 1
            self._fist_off_count = 0

            if self._fist_on_count >= self._fist_confirm_frames:
                self._stable_fist = True
        else:
            self._fist_off_count += 1
            self._fist_on_count = 0

            if self._fist_off_count >= self._fist_release_frames:
                self._stable_fist = False

        return self._stable_fist

    def _publish_hand_state(
        self,
        detected: bool,
        center: Optional[NormalizedPoint],
        fist: bool,
        position_z: float = 0.0,
    ) -> None:
        """한 프레임의 손 상태를 ROS 토픽으로 발행합니다."""

        self._detected_publisher.publish(Bool(data=detected))
        self._fist_publisher.publish(Bool(data=fist))

        # 손을 찾지 못한 경우 잘못된 (0, 0)을 위치로 발행하지 않습니다.
        # 로봇 제어 노드는 hand_detected=False를 받으면 즉시 추종을 멈추고,
        # 마지막 위치 메시지의 시간도 watchdog으로 확인하게 만들 예정입니다.
        if not detected or center is None:
            return

        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._position_frame_id
        message.point.x = float(center[0])
        message.point.y = float(center[1])
        message.point.z = float(position_z)
        self._position_publisher.publish(message)

    def _handle_no_hand(self) -> None:
        """손을 찾지 못했을 때 필터와 안전 관련 상태를 초기화합니다."""

        # 손을 놓친 상태에서 이전 좌표를 계속 이어 쓰면 재검출 순간 로봇 목표가
        # 튈 수 있으므로 중심 필터를 초기화합니다.
        self._filtered_center = None

        # 주먹 상태는 안전과 직접 관련되므로 손을 잃으면 즉시 false로 만듭니다.
        self._fist_on_count = 0
        self._fist_off_count = 0
        self._stable_fist = False

        self._publish_hand_state(detected=False, center=None, fist=False)

    def _process_frame(self) -> None:
        """타이머마다 웹캠 한 프레임을 읽고 검출·발행·표시합니다."""

        success, frame = self._capture.read()
        if not success or frame is None:
            self._handle_no_hand()

            # 같은 오류는 2초에 한 번만 출력합니다.
            now = time.monotonic()
            if now - self._last_camera_error_log_time >= 2.0:
                self.get_logger().error(
                    "웹캠 프레임을 읽지 못했습니다. 카메라 연결을 확인하세요."
                )
                self._last_camera_error_log_time = now
            return

        if self._mirror_image:
            frame = cv2.flip(frame, 1)

        # OpenCV는 BGR, MediaPipe는 RGB 순서를 사용합니다.
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame = np.ascontiguousarray(rgb_frame)

        mediapipe_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_frame,
        )

        result = self._landmarker.detect_for_video(
            mediapipe_image,
            self._next_mediapipe_timestamp_ms(),
        )

        landmarks = None
        raw_center = None
        filtered_center = None
        raw_fist = False
        folded_count = 0
        handedness_text = "Unknown"

        if result.hand_landmarks:
            # num_hands=1로 설정했으므로 첫 번째 손만 사용합니다.
            landmarks = result.hand_landmarks[0]
            raw_center = calculate_palm_center(landmarks)
            filtered_center = self._low_pass_filter(raw_center)

            raw_fist, folded_count = estimate_fist(
                landmarks,
                tip_distance_ratio=self._fist_tip_distance_ratio,
                minimum_folded_fingers=self._fist_min_folded_fingers,
            )
            stable_fist = self._update_stable_fist(raw_fist)

            if result.handedness and result.handedness[0]:
                category = result.handedness[0][0]
                label = category.category_name or "Unknown"
                score = float(category.score or 0.0)
                handedness_text = f"{label} {score:.2f}"

            self._publish_hand_state(
                detected=True,
                center=filtered_center,
                fist=stable_fist,
            )
        else:
            self._handle_no_hand()
            stable_fist = False

        if self._show_debug_window:
            self._draw_debug_view(
                frame=frame,
                landmarks=landmarks,
                raw_center=raw_center,
                filtered_center=filtered_center,
                raw_fist=raw_fist,
                stable_fist=stable_fist,
                folded_count=folded_count,
                handedness_text=handedness_text,
            )

    def _draw_debug_view(
        self,
        frame: np.ndarray,
        landmarks: Optional[Sequence[Any]],
        raw_center: Optional[NormalizedPoint],
        filtered_center: Optional[NormalizedPoint],
        raw_fist: bool,
        stable_fist: bool,
        folded_count: int,
        handedness_text: str,
        published_z: Optional[float] = None,
        display_window: bool = True,
    ) -> None:
        """손 랜드마크와 판정 상태를 그리고 필요할 때 OpenCV 창에 표시합니다.

        ``frame`` 자체에 주석을 그리므로, ``display_window=False``로 호출한
        뒤에도 호출자는 완성된 주석 영상을 ROS 이미지 토픽 등으로 사용할 수
        있습니다.
        """

        height, width = frame.shape[:2]

        # 영상 중앙을 표시합니다. 이것은 손 기준점이 아니라 단순 화면 중심입니다.
        cv2.drawMarker(
            frame,
            (width // 2, height // 2),
            (255, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=1,
        )

        if landmarks is not None:
            # 손가락 뼈대 선
            for start_index, end_index in HAND_CONNECTIONS:
                start = landmarks[start_index]
                end = landmarks[end_index]
                start_pixel = (
                    int(_clamp(float(start.x), 0.0, 1.0) * (width - 1)),
                    int(_clamp(float(start.y), 0.0, 1.0) * (height - 1)),
                )
                end_pixel = (
                    int(_clamp(float(end.x), 0.0, 1.0) * (width - 1)),
                    int(_clamp(float(end.y), 0.0, 1.0) * (height - 1)),
                )
                cv2.line(frame, start_pixel, end_pixel, (80, 220, 80), 2)

            # 21개 랜드마크 점
            for landmark in landmarks:
                pixel = (
                    int(_clamp(float(landmark.x), 0.0, 1.0) * (width - 1)),
                    int(_clamp(float(landmark.y), 0.0, 1.0) * (height - 1)),
                )
                cv2.circle(frame, pixel, 3, (0, 140, 255), -1)

                # 디버깅 시 랜드마크 번호를 보고 싶으면 아래 줄을 활성화할 수 있습니다.
                # cv2.putText(frame, str(index), pixel, 0, 0.35, (255, 255, 255), 1)

        # 노란 원: 필터를 적용하기 전의 손바닥 중심
        if raw_center is not None:
            raw_pixel = (
                int(raw_center[0] * (width - 1)),
                int(raw_center[1] * (height - 1)),
            )
            cv2.circle(frame, raw_pixel, 7, (0, 255, 255), 2)

        # 초록 십자: ROS 토픽으로 실제 발행하는 필터 적용 후 손바닥 중심
        if filtered_center is not None:
            filtered_pixel = (
                int(filtered_center[0] * (width - 1)),
                int(filtered_center[1] * (height - 1)),
            )
            cv2.drawMarker(
                frame,
                filtered_pixel,
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=24,
                thickness=2,
            )

        status_color = (0, 0, 255) if stable_fist else (0, 255, 0)
        stable_text = "FIST" if stable_fist else "OPEN / NOT CONFIRMED"

        cv2.putText(
            frame,
            f"hand={landmarks is not None}  {handedness_text}",
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"raw_fist={raw_fist} folded={folded_count}/4",
            (15, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"stable={stable_text}",
            (15, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            status_color,
            2,
            cv2.LINE_AA,
        )

        if filtered_center is not None:
            position_text = (
                f"published x={filtered_center[0]:.3f}, "
                f"y={filtered_center[1]:.3f}"
            )
            if published_z is not None:
                position_text += f", z={published_z:+.3f}"
            cv2.putText(
                frame,
                position_text,
                (15, 112),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            frame,
            "Q / ESC: quit",
            (15, height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

        # UI에서 ROS 이미지 토픽만 사용할 때는 주석까지 그린 뒤 로컬 GUI는
        # 열지 않습니다. 영상 생성과 imshow를 분리하면 headless 환경에서도
        # 같은 주석 영상을 발행할 수 있습니다.
        if not display_window:
            return

        try:
            cv2.imshow(self._window_name, frame)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            # GUI가 없는 환경에서 imshow가 실패해도 토픽 발행은 계속합니다.
            self.get_logger().error(
                f"OpenCV 디버그 창을 열 수 없어 화면 표시를 끕니다: {error}"
            )
            self._show_debug_window = False
            return

        if key in (ord("q"), ord("Q"), 27):
            self.get_logger().info("사용자 요청으로 손 추적 노드를 종료합니다.")
            self._timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()

    def _cleanup(self) -> None:
        """카메라와 MediaPipe 리소스를 안전하게 한 번만 해제합니다."""

        if self._cleaned_up:
            return
        self._cleaned_up = True

        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.cancel()

        capture = getattr(self, "_capture", None)
        if capture is not None:
            capture.release()

        landmarker = getattr(self, "_landmarker", None)
        if landmarker is not None:
            try:
                landmarker.close()
            except RuntimeError as error:
                self.get_logger().warning(
                    f"MediaPipe 종료 중 경고가 발생했습니다: {error}"
                )

        if getattr(self, "_show_debug_window", False):
            try:
                cv2.destroyWindow(self._window_name)
            except cv2.error:
                # 창이 이미 닫혔거나 생성되지 않은 경우는 무시합니다.
                pass

    def destroy_node(self) -> bool:
        """ROS 노드가 종료될 때 카메라와 창까지 함께 정리합니다."""

        self._cleanup()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    """ROS 2 console_scripts에서 호출하는 진입 함수입니다."""

    rclpy.init(args=args)
    node: Optional[HandTrackerNode] = None

    try:
        node = HandTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 터미널 Ctrl+C는 정상 종료로 처리합니다.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
