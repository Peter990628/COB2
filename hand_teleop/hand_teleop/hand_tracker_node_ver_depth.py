#!/usr/bin/env python3
"""2D 웹캠 손 추적에 상대 깊이 추정을 추가한 ROS 2 노드입니다.

이 노드는 기존 ``hand_tracker_node``와 같은 X/Y 및 주먹 상태를 발행하면서,
손바닥이 화면에서 차지하는 크기를 이용해 카메라에 가까워졌는지/멀어졌는지를
추정합니다.

발행 토픽
----------
* /hand_teleop/hand_position (geometry_msgs/PointStamped)
    - point.x: 화면 왼쪽 0.0 / 오른쪽 1.0
    - point.y: 화면 위쪽 0.0 / 아래쪽 1.0
    - point.z: 중립 거리 기준 상대 깊이 -1.0~+1.0
      +1.0 쪽은 손이 카메라에 가까워진 상태이고,
      -1.0 쪽은 손이 카메라에서 멀어진 상태입니다.
* /hand_teleop/hand_detected (std_msgs/Bool)
* /hand_teleop/fist (std_msgs/Bool)
* /hand_teleop/depth_valid (std_msgs/Bool)
    - 중립 손 크기 보정이 끝나 현재 Z를 사용할 수 있는지 나타냅니다.
* /hand_teleop/annotated_image (sensor_msgs/Image)
    - 웹캠 영상에 손 랜드마크, 중심점, 주먹 및 깊이 상태를 그린 BGR8
      영상입니다. UI에서는 이 토픽을 구독해 로컬 OpenCV 창과 같은 내용을
      표시할 수 있습니다.

중요
----
일반 2D 웹캠에는 거리 센서가 없으므로 point.z는 mm 단위 실제 거리가 아닙니다.
MediaPipe 랜드마크의 자체 z도 카메라까지의 절대 거리가 아니기 때문에, 여기서는
손바닥의 화면상 크기를 사용합니다. 손을 카메라 쪽으로 밀면 손바닥이 커져 Z가
양수가 되고, 뒤로 빼면 작아져 Z가 음수가 됩니다.

노드를 시작한 직후에는 편 손을 평소 조종할 중립 거리에 잠시 유지해야 합니다.
기본 45개 프레임의 중앙값이 중립 손 크기로 저장됩니다. 손 크기와 카메라 위치가
달라지면 노드를 다시 시작해 보정하는 것이 가장 간단합니다.

이 노드는 로봇을 직접 움직이지 않습니다. 이후 ``hand_follow_robot_node``에서
아래와 같이 제한된 BASE X 목표로 변환해야 합니다.

    robot_x = follow_center_x - hand_z * allowed_x_half_range_mm

즉, 손이 카메라 쪽으로 가까워져 hand_z가 양수가 되면 로봇은 BASE -X 쪽으로
이동합니다. 반드시 로봇 X 최소/최대 범위로 한 번 더 제한해야 합니다.
"""
# hand_tracker_node_ver_depth

from __future__ import annotations

import math
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from hand_teleop.hand_tracker_node import (
    HandTrackerNode,
    NormalizedPoint,
    _clamp,
    calculate_palm_center,
    estimate_fist,
)


# 손바닥 폭은 검지 MCP(5)와 소지 MCP(17), 손바닥 높이는 손목(0)과
# 중지 MCP(9) 사이 거리로 계산합니다. 손가락 끝을 사용하지 않기 때문에
# 손가락을 펴거나 접을 때 생기는 크기 변화의 영향을 비교적 적게 받습니다.
PALM_WIDTH_LANDMARKS = (5, 17)
PALM_HEIGHT_LANDMARKS = (0, 9)


def _landmark_distance_px(
    landmarks: Sequence[Any],
    first_index: int,
    second_index: int,
    frame_width: int,
    frame_height: int,
) -> float:
    """두 랜드마크 사이의 2D 픽셀 거리를 반환합니다."""

    first = landmarks[first_index]
    second = landmarks[second_index]
    delta_x = (float(first.x) - float(second.x)) * float(frame_width)
    delta_y = (float(first.y) - float(second.y)) * float(frame_height)
    return float(math.hypot(delta_x, delta_y))


def estimate_palm_size_px(
    landmarks: Sequence[Any],
    frame_width: int,
    frame_height: int,
) -> Tuple[float, float, float]:
    """손바닥 폭·높이와 이 둘을 합친 대표 크기를 픽셀 단위로 계산합니다.

    폭 또는 높이 하나만 사용하면 손이 카메라에 대해 기울었을 때 값이 많이
    바뀔 수 있습니다. 두 값의 기하평균을 사용하면 한 방향의 작은 흔들림에
    조금 더 안정적입니다.
    """

    palm_width_px = _landmark_distance_px(
        landmarks,
        PALM_WIDTH_LANDMARKS[0],
        PALM_WIDTH_LANDMARKS[1],
        frame_width,
        frame_height,
    )
    palm_height_px = _landmark_distance_px(
        landmarks,
        PALM_HEIGHT_LANDMARKS[0],
        PALM_HEIGHT_LANDMARKS[1],
        frame_width,
        frame_height,
    )

    if palm_width_px <= 0.0 or palm_height_px <= 0.0:
        return palm_width_px, palm_height_px, 0.0

    palm_size_px = float(math.sqrt(palm_width_px * palm_height_px))
    return palm_width_px, palm_height_px, palm_size_px


class DepthHandTrackerNode(HandTrackerNode):
    """기존 손 추적기에 손 크기 기반 상대 깊이를 추가합니다."""

    def __init__(self) -> None:
        # 부모 생성자가 카메라, MediaPipe, X/Y 필터, 주먹 판정과 ROS 토픽을
        # 준비합니다. 기본 노드와 동시에 실행할 때 이름이 충돌하지 않게 별도의
        # 노드 이름을 사용합니다.
        super().__init__(node_name="hand_tracker_node_ver_depth")
        self._position_frame_id = "webcam_normalized_relative_depth"

        # ------------------------------------------------------------------
        # 상대 깊이 전용 ROS 파라미터
        # ------------------------------------------------------------------
        # 자동 중립 보정에 사용할 유효 프레임 수입니다. 30 fps에서 45장은
        # 약 1.5초입니다. 보정 중에는 손을 펴고 평소 조종 거리에 둡니다.
        self.declare_parameter("depth_calibration_frames", 45)

        # 0보다 큰 값을 지정하면 자동 보정 대신 해당 픽셀 크기를 중립값으로
        # 사용합니다. 카메라 해상도나 사용자가 바뀌면 다시 측정해야 합니다.
        self.declare_parameter("depth_reference_palm_size_px", 0.0)

        # 중립 손 크기에서 이 비율만큼 커지거나 작아지면 Z 절댓값이 1.0이
        # 됩니다. 예: 0.35이면 중립보다 35% 커질 때 대략 +1.0입니다.
        self.declare_parameter("depth_full_scale_ratio", 0.35)

        # 손 검출의 미세한 크기 떨림으로 로봇 X가 흔들리지 않게 만드는
        # 중립 구간입니다. 0.04는 중립 크기의 ±4%를 Z=0으로 봅니다.
        self.declare_parameter("depth_deadzone_ratio", 0.04)

        # Z 전용 지수 이동 평균 필터 계수입니다. 작을수록 부드럽지만 느립니다.
        self.declare_parameter("depth_filter_alpha", 0.20)

        # 너무 작거나 비정상적으로 큰 손 검출값을 깊이에 사용하지 않습니다.
        self.declare_parameter("depth_min_palm_size_px", 20.0)
        self.declare_parameter("depth_max_palm_size_px", 400.0)

        # 보정 표본의 10~90 백분위 폭이 중립값의 이 비율보다 크면 사용자가
        # 앞뒤로 움직인 것으로 판단해 자동 보정을 다시 시작합니다.
        self.declare_parameter("depth_calibration_max_spread_ratio", 0.15)

        self.declare_parameter(
            "depth_valid_topic",
            "/hand_teleop/depth_valid",
        )

        # ------------------------------------------------------------------
        # UI용 주석 영상 토픽 설정
        # ------------------------------------------------------------------
        # True이면 손 랜드마크와 상태 글자가 그려진 최종 BGR 영상을
        # sensor_msgs/Image로 발행합니다. show_debug_window=False여도 이
        # 토픽은 계속 발행되므로 통합 UI만 띄우는 구성에 사용할 수 있습니다.
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter(
            "annotated_image_topic",
            "/hand_teleop/annotated_image",
        )
        self.declare_parameter(
            "annotated_image_frame_id",
            "webcam_annotated",
        )

        self._depth_calibration_frames = int(
            self.get_parameter("depth_calibration_frames").value
        )
        configured_reference = float(
            self.get_parameter("depth_reference_palm_size_px").value
        )
        self._depth_full_scale_ratio = float(
            self.get_parameter("depth_full_scale_ratio").value
        )
        self._depth_deadzone_ratio = float(
            self.get_parameter("depth_deadzone_ratio").value
        )
        self._depth_filter_alpha = float(
            self.get_parameter("depth_filter_alpha").value
        )
        self._depth_min_palm_size_px = float(
            self.get_parameter("depth_min_palm_size_px").value
        )
        self._depth_max_palm_size_px = float(
            self.get_parameter("depth_max_palm_size_px").value
        )
        self._depth_calibration_max_spread_ratio = float(
            self.get_parameter(
                "depth_calibration_max_spread_ratio"
            ).value
        )

        if self._depth_calibration_frames < 5:
            raise ValueError("depth_calibration_frames는 5 이상이어야 합니다.")
        if self._depth_full_scale_ratio <= 0.0:
            raise ValueError("depth_full_scale_ratio는 0보다 커야 합니다.")
        if not 0.0 <= self._depth_deadzone_ratio < self._depth_full_scale_ratio:
            raise ValueError(
                "depth_deadzone_ratio는 0 이상이고 "
                "depth_full_scale_ratio보다 작아야 합니다."
            )
        if not 0.0 < self._depth_filter_alpha <= 1.0:
            raise ValueError("depth_filter_alpha는 0보다 크고 1 이하여야 합니다.")
        if (
            self._depth_min_palm_size_px <= 0.0
            or self._depth_max_palm_size_px
            <= self._depth_min_palm_size_px
        ):
            raise ValueError("손바닥 픽셀 크기 허용 범위가 잘못되었습니다.")
        if (
            configured_reference > 0.0
            and not (
                self._depth_min_palm_size_px
                <= configured_reference
                <= self._depth_max_palm_size_px
            )
        ):
            raise ValueError(
                "depth_reference_palm_size_px가 손바닥 픽셀 크기 "
                "허용 범위 밖입니다."
            )
        if self._depth_calibration_max_spread_ratio <= 0.0:
            raise ValueError(
                "depth_calibration_max_spread_ratio는 0보다 커야 합니다."
            )

        self._depth_reference_palm_size_px: Optional[float] = (
            configured_reference if configured_reference > 0.0 else None
        )
        self._depth_calibration_samples: List[float] = []
        self._filtered_depth: Optional[float] = None

        depth_valid_topic = str(
            self.get_parameter("depth_valid_topic").value
        )
        self._depth_valid_publisher = self.create_publisher(
            Bool,
            depth_valid_topic,
            10,
        )

        self._publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self._annotated_image_topic = str(
            self.get_parameter("annotated_image_topic").value
        )
        self._annotated_image_frame_id = str(
            self.get_parameter("annotated_image_frame_id").value
        )
        if (
            self._publish_annotated_image
            and not self._annotated_image_topic.strip()
        ):
            raise ValueError(
                "publish_annotated_image=True이면 "
                "annotated_image_topic이 비어 있으면 안 됩니다."
            )

        # 이미지가 UI 처리 속도보다 빠르게 도착해도 오래된 프레임이 쌓이지
        # 않도록 큐 깊이를 1로 둡니다. sensor_msgs/Image를 직접 구성하므로
        # 현재 NumPy 2 환경과 ROS Humble cv_bridge 사이의 ABI 충돌도
        # 피할 수 있습니다.
        self._annotated_image_publisher = (
            self.create_publisher(
                Image,
                self._annotated_image_topic,
                1,
            )
            if self._publish_annotated_image
            else None
        )

        if self._depth_reference_palm_size_px is None:
            self.get_logger().info(
                "상대 깊이 중립 보정을 시작합니다. "
                f"편 손을 평소 거리에서 "
                f"{self._depth_calibration_frames}프레임 동안 유지하세요."
            )
        else:
            self.get_logger().info(
                "설정된 상대 깊이 중립값을 사용합니다: "
                f"palm_size={self._depth_reference_palm_size_px:.1f}px"
            )
        self.get_logger().info(
            "깊이 규칙: 카메라 쪽=+Z, 카메라 반대쪽=-Z, "
            f"범위=-1.0~+1.0, valid={depth_valid_topic}"
        )
        if self._annotated_image_publisher is not None:
            self.get_logger().info(
                "UI용 주석 영상을 발행합니다: "
                f"topic={self._annotated_image_topic}, "
                "encoding=bgr8"
            )

    def _is_valid_palm_size(self, palm_size_px: float) -> bool:
        """손바닥 크기가 깊이 계산에 사용할 수 있는 범위인지 확인합니다."""

        return (
            math.isfinite(palm_size_px)
            and self._depth_min_palm_size_px
            <= palm_size_px
            <= self._depth_max_palm_size_px
        )

    def _try_finish_depth_calibration(self) -> bool:
        """표본이 충분하면 중립 손 크기를 확정합니다."""

        if (
            len(self._depth_calibration_samples)
            < self._depth_calibration_frames
        ):
            return False

        samples = np.asarray(
            self._depth_calibration_samples,
            dtype=np.float64,
        )
        reference = float(np.median(samples))
        lower = float(np.percentile(samples, 10.0))
        upper = float(np.percentile(samples, 90.0))
        relative_spread = (
            float((upper - lower) / reference)
            if reference > 0.0
            else math.inf
        )

        if (
            not self._is_valid_palm_size(reference)
            or relative_spread
            > self._depth_calibration_max_spread_ratio
        ):
            self.get_logger().warning(
                "중립 깊이 보정 중 손 크기 변화가 너무 컸습니다: "
                f"median={reference:.1f}px, "
                f"spread={relative_spread:.3f}. "
                "손을 펴고 앞뒤로 움직이지 않은 채 다시 유지하세요."
            )
            self._depth_calibration_samples.clear()
            return False

        self._depth_reference_palm_size_px = reference
        self._filtered_depth = 0.0
        self.get_logger().info(
            "상대 깊이 중립 보정 완료: "
            f"palm_size={reference:.1f}px, "
            f"spread={relative_spread:.3f}"
        )
        return True

    def _calculate_depth(
        self,
        palm_size_px: float,
        allow_calibration_sample: bool,
    ) -> Tuple[Optional[float], Optional[float], bool]:
        """손 크기에서 필터 전/후 상대 깊이와 유효 여부를 계산합니다."""

        if not self._is_valid_palm_size(palm_size_px):
            self._filtered_depth = None
            return None, None, False

        # 편 손으로 판단되는 프레임만 중립 보정에 사용합니다. 주먹을 쥐면
        # 랜드마크가 겹쳐 손바닥 크기 추정이 흔들릴 수 있습니다.
        if self._depth_reference_palm_size_px is None:
            if allow_calibration_sample:
                self._depth_calibration_samples.append(palm_size_px)
                self._depth_calibration_samples = (
                    self._depth_calibration_samples[
                        -self._depth_calibration_frames:
                    ]
                )
                self._try_finish_depth_calibration()

            if self._depth_reference_palm_size_px is None:
                return None, None, False

        reference = self._depth_reference_palm_size_px
        assert reference is not None

        # 손이 가까워져 화면에서 커지면 size_delta가 양수가 됩니다.
        size_delta_ratio = float(palm_size_px / reference - 1.0)
        magnitude = abs(size_delta_ratio)

        if magnitude <= self._depth_deadzone_ratio:
            raw_depth = 0.0
        else:
            # 데드존 경계에서 값이 갑자기 튀지 않도록 데드존만큼 뺀 뒤
            # 남은 전체 범위로 다시 정규화합니다.
            usable_range = (
                self._depth_full_scale_ratio
                - self._depth_deadzone_ratio
            )
            raw_magnitude = (
                magnitude - self._depth_deadzone_ratio
            ) / usable_range
            raw_depth = math.copysign(raw_magnitude, size_delta_ratio)
            raw_depth = _clamp(raw_depth, -1.0, 1.0)

        if self._filtered_depth is None:
            filtered_depth = raw_depth
        else:
            alpha = self._depth_filter_alpha
            filtered_depth = (
                alpha * raw_depth
                + (1.0 - alpha) * self._filtered_depth
            )

        self._filtered_depth = _clamp(filtered_depth, -1.0, 1.0)
        return raw_depth, self._filtered_depth, True

    def _publish_depth_valid(self, valid: bool) -> None:
        """현재 프레임의 상대 깊이를 사용할 수 있는지 발행합니다."""

        self._depth_valid_publisher.publish(Bool(data=valid))

    def _publish_annotated_frame(self, frame: np.ndarray) -> None:
        """주석이 모두 그려진 BGR 프레임을 ROS ``Image``로 발행합니다.

        ``cv_bridge``를 사용하지 않고 메시지 필드를 직접 채웁니다. 현재
        프로젝트의 NumPy 2.x 환경에서는 Ubuntu 22.04/ROS Humble에 포함된
        cv_bridge가 NumPy 1.x ABI로 빌드되어 충돌할 수 있기 때문입니다.
        """

        publisher = self._annotated_image_publisher
        if publisher is None:
            return

        image = np.ascontiguousarray(frame, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(
                "주석 영상은 HxWx3 BGR 배열이어야 합니다."
            )

        height, width = image.shape[:2]
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._annotated_image_frame_id
        message.height = int(height)
        message.width = int(width)
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = int(width * 3)
        message.data = image.tobytes()
        publisher.publish(message)

    def _handle_no_hand(self) -> None:
        """손을 놓치면 Z 필터를 초기화하고 깊이를 무효화합니다."""

        self._filtered_depth = None
        self._publish_depth_valid(False)
        super()._handle_no_hand()

    def _draw_depth_debug(
        self,
        frame: np.ndarray,
        palm_width_px: Optional[float],
        palm_height_px: Optional[float],
        palm_size_px: Optional[float],
        raw_depth: Optional[float],
        filtered_depth: Optional[float],
        depth_valid: bool,
    ) -> None:
        """기존 디버그 영상에 깊이 보정·추정 상태를 추가로 표시합니다."""

        if self._depth_reference_palm_size_px is None:
            progress = len(self._depth_calibration_samples)
            first_line = (
                "depth=CALIBRATING "
                f"{progress}/{self._depth_calibration_frames}"
            )
            second_line = "hold OPEN hand at neutral distance"
            color = (0, 200, 255)
        elif depth_valid and filtered_depth is not None:
            direction = (
                "NEAR"
                if filtered_depth > 0.02
                else "FAR"
                if filtered_depth < -0.02
                else "NEUTRAL"
            )
            raw_text = (
                "N/A" if raw_depth is None else f"{raw_depth:+.3f}"
            )
            first_line = (
                f"depth raw={raw_text} "
                f"filtered={filtered_depth:+.3f} {direction}"
            )
            second_line = (
                f"palm={palm_size_px:.1f}px "
                f"reference={self._depth_reference_palm_size_px:.1f}px"
                if palm_size_px is not None
                else "palm=N/A"
            )
            color = (255, 200, 0)
        else:
            first_line = "depth=INVALID"
            second_line = "show the whole palm to the camera"
            color = (0, 0, 255)

        cv2.putText(
            frame,
            first_line,
            (15, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            second_line,
            (15, 168),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color,
            2,
            cv2.LINE_AA,
        )

        if palm_width_px is not None and palm_height_px is not None:
            cv2.putText(
                frame,
                (
                    f"palm width={palm_width_px:.1f}px "
                    f"height={palm_height_px:.1f}px"
                ),
                (15, 194),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )

        # 화면 아래에 -1(멀어짐)~+1(가까워짐) 깊이 막대를 표시합니다.
        if filtered_depth is not None:
            height, width = frame.shape[:2]
            bar_left = 80
            bar_right = max(bar_left + 10, width - 80)
            bar_y = height - 42
            bar_center = (bar_left + bar_right) // 2
            marker_x = int(
                round(
                    bar_center
                    + filtered_depth
                    * (bar_right - bar_left)
                    / 2.0
                )
            )
            cv2.line(
                frame,
                (bar_left, bar_y),
                (bar_right, bar_y),
                (180, 180, 180),
                2,
            )
            cv2.line(
                frame,
                (bar_center, bar_y - 7),
                (bar_center, bar_y + 7),
                (255, 255, 255),
                1,
            )
            cv2.circle(
                frame,
                (marker_x, bar_y),
                6,
                color,
                -1,
            )
            cv2.putText(
                frame,
                "FAR",
                (bar_left - 48, bar_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                "NEAR",
                (bar_right + 8, bar_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )

    def _process_frame(self) -> None:
        """웹캠 한 프레임에서 X/Y, 주먹과 상대 Z를 함께 계산합니다."""

        success, frame = self._capture.read()
        if not success or frame is None:
            self._handle_no_hand()

            # 부모 노드와 같은 카메라 오류 로그 제한을 사용합니다.
            import time

            now = time.monotonic()
            if now - self._last_camera_error_log_time >= 2.0:
                self.get_logger().error(
                    "웹캠 프레임을 읽지 못했습니다. 카메라 연결을 확인하세요."
                )
                self._last_camera_error_log_time = now
            return

        if self._mirror_image:
            frame = cv2.flip(frame, 1)

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
        raw_center: Optional[NormalizedPoint] = None
        filtered_center: Optional[NormalizedPoint] = None
        raw_fist = False
        stable_fist = False
        folded_count = 0
        handedness_text = "Unknown"

        palm_width_px: Optional[float] = None
        palm_height_px: Optional[float] = None
        palm_size_px: Optional[float] = None
        raw_depth: Optional[float] = None
        filtered_depth: Optional[float] = None
        depth_valid = False

        if result.hand_landmarks:
            landmarks = result.hand_landmarks[0]
            raw_center = calculate_palm_center(landmarks)
            filtered_center = self._low_pass_filter(raw_center)

            raw_fist, folded_count = estimate_fist(
                landmarks,
                tip_distance_ratio=self._fist_tip_distance_ratio,
                minimum_folded_fingers=self._fist_min_folded_fingers,
            )
            stable_fist = self._update_stable_fist(raw_fist)

            height, width = frame.shape[:2]
            (
                palm_width_px,
                palm_height_px,
                palm_size_px,
            ) = estimate_palm_size_px(
                landmarks,
                width,
                height,
            )
            (
                raw_depth,
                filtered_depth,
                depth_valid,
            ) = self._calculate_depth(
                palm_size_px,
                # 편 손 또는 손가락 하나 정도만 순간적으로 접힌 프레임을
                # 중립 보정에 사용합니다.
                allow_calibration_sample=(
                    not raw_fist and folded_count <= 1
                ),
            )

            if result.handedness and result.handedness[0]:
                category = result.handedness[0][0]
                label = category.category_name or "Unknown"
                score = float(category.score or 0.0)
                handedness_text = f"{label} {score:.2f}"

            # 보정 전에도 X/Y는 사용할 수 있게 계속 발행하되, Z는 0으로
            # 발행하고 depth_valid=False로 명확하게 구분합니다.
            published_depth = (
                float(filtered_depth)
                if depth_valid and filtered_depth is not None
                else 0.0
            )
            self._publish_hand_state(
                detected=True,
                center=filtered_center,
                fist=stable_fist,
                position_z=published_depth,
            )
            self._publish_depth_valid(depth_valid)
        else:
            self._handle_no_hand()

        # 로컬 OpenCV 창과 ROS UI 영상 발행을 서로 독립적으로 사용할 수
        # 있도록, 둘 중 하나라도 필요하면 동일한 최종 주석 프레임을 만듭니다.
        if self._show_debug_window or self._publish_annotated_image:
            self._draw_depth_debug(
                frame,
                palm_width_px,
                palm_height_px,
                palm_size_px,
                raw_depth,
                filtered_depth,
                depth_valid,
            )
            self._draw_debug_view(
                frame=frame,
                landmarks=landmarks,
                raw_center=raw_center,
                filtered_center=filtered_center,
                raw_fist=raw_fist,
                stable_fist=stable_fist,
                folded_count=folded_count,
                handedness_text=handedness_text,
                published_z=(
                    filtered_depth
                    if depth_valid and filtered_depth is not None
                    else 0.0
                ),
                display_window=self._show_debug_window,
            )
            if rclpy.ok():
                self._publish_annotated_frame(frame)


def main(args: Optional[Sequence[str]] = None) -> None:
    """ROS 2 console_scripts에서 호출하는 진입 함수입니다."""

    rclpy.init(args=args)
    node: Optional[DepthHandTrackerNode] = None

    try:
        node = DepthHandTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
