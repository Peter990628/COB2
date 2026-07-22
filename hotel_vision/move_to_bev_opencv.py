"""OpenCV로 캔 중심을 보정해 수직 파지하고 트레이에 배치하는 ROS 2 노드."""
# move_to_bev_opencv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from checkin_interfaces.msg import BeveragePosition
from cv_bridge import CvBridge
from hotel_vision.gripper.onrobot import RG
from hotel_vision.opencv_center_test import (
    CAMERA_INFO_TOPIC,
    COLOR_TOPIC,
    DEPTH_TOPIC,
    RGB_DEPTH_MAX_STAMP_DIFF_SEC,
    ROTATE_CAMERA_180,
    CircleObservation,
    detect_can_top_circle,
    draw_detection_overlay,
    message_stamp_sec,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

import DR_init


# ---------------------------------------------------------------------------
# 실행 전에 주로 수정할 설정값
# ---------------------------------------------------------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
BEVERAGE_POSITION_TOPIC = "/checkin/beverage_position"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 첫 번째 음료 좌표를 받기 전에 이동하는 관절 자세 [J1 ... J6] (deg)
INITIAL_JOINT_POSE = [23.559, 16.219, 97.41, -51.183, 95.124, 210.84]

# 같은 XYZ에서 카메라/그리퍼가 아래를 보게 만드는 자세 [A, B, C] (deg)
VERTICAL_TOOL_ABC = [90.0, -180.0, -90.0]

# detector가 준 캔 좌표보다 BASE +Z 방향으로 올라갈 거리 (mm)
ROUGH_APPROACH_Z_OFFSET_MM = 80.0 
SAFE_CLEARANCE_ABOVE_CAN_MM = 100.0

# 검출된 캔 윗면보다 최종 TCP가 얼마나 위에 있어야 하는지 정하는 값이다.
# TCP가 실제 그리퍼 파지 중심에 설정되어 있는지 확인한 후 보정해야 한다.
# 최종 하강량은 현재 TCP Z - (검출된 캔 윗면 Z + 이 값)으로 계산된다.
GRASP_TCP_Z_OFFSET_FROM_CAN_TOP_MM = -30.0
MIN_FINAL_DESCENT_MM = 5.0
MAX_FINAL_DESCENT_MM = 120.0

# True이면 최종 하강과 파지를 실행한다. ROI와 좌표를 먼저 확인하려면
# False로 두고, 충분히 검증한 뒤에만 True로 바꾼다.
ENABLE_FINAL_DESCENT_AND_GRIP = True

# 파지 후 beverage_test_srv와 같은 트레이 배치 작업까지 실행할지 정한다.
# 실제 로봇에서 True로 실행하기 전에 아래 접근 pose와 release Z를 반드시
# 티칭한 트레이 좌표와 대조한다.
ENABLE_TRAY_PLACE = True

# 트레이 위의 안전한 접근 TCP pose [X, Y, Z, A, B, C] (BASE, mm/deg)
PLACE_APPROACH_POSE = [
    281.662,
    43.451,
    173.977,
    16.839,
    -178.666,
    16.09,
]

# PLACE_APPROACH_POSE의 X/Y/A/B/C를 유지한 채 캔을 놓을 TCP의 BASE Z
PLACE_RELEASE_Z_MM = 110.0

# 캔을 들고 수평 이동하거나 놓은 뒤 빠져나올 때 release Z보다 확보할 높이
PLACE_RETREAT_CLEARANCE_MM = 150.0

# True이면 배치 후 안전 높이까지 후퇴한 다음 초기 관절 자세로 돌아간다.
RETURN_TO_INITIAL_POSE_AFTER_PLACE = False

JOINT_VELOCITY = 30.0
JOINT_ACCELERATION = 30.0
LINEAR_VELOCITY = 20.0
LINEAR_ACCELERATION = 20.0
FINE_LINEAR_VELOCITY = 5.0
FINE_LINEAR_ACCELERATION = 5.0

# 수직 정렬 완료 후 카메라 흔들림이 가라앉을 때까지 기다리는 시간
CAMERA_SETTLE_SEC = 0.8
CAMERA_READY_TIMEOUT_SEC = 5.0
VISION_TIMEOUT_SEC = 15.0
VISION_REQUIRED_SAMPLES = 5
VISION_CENTER_MAX_SPREAD_PX = 8.0
VISION_DEPTH_MAX_RANGE_MM = 15.0
VISION_RADIUS_MAX_RELATIVE_DEVIATION = 0.20

# 거친 위치와 OpenCV 정밀 위치의 XY 차이가 이보다 크면 잘못된 원으로 본다.
MAX_XY_CORRECTION_MM = 80.0
MIN_CAN_VERTICAL_GAP_MM = 20.0
MAX_CAN_VERTICAL_GAP_MM = 300.0

# 거친 detector 좌표 자체에 적용하는 1차 안전 범위이다. 실제 작업 셀의
# 펜스/테이블 범위가 더 좁다면 반드시 이 값도 더 좁게 설정한다.
MIN_ROUGH_CONFIDENCE = 0.50
BASE_X_RANGE_MM = (-800.0, 800.0)
BASE_Y_RANGE_MM = (-800.0, 800.0)
BASE_Z_RANGE_MM = (-100.0, 900.0)
MAX_BASE_XY_RADIUS_MM = 850.0

# OnRobot RG2 raw 단위: width=0.1 mm, force=0.1 N.
# 아래 현재값은 파지 폭 50.0 mm, 파지력 20.0 N이다.
GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502
GRIPPER_OPEN_WIDTH_RAW = 700
GRIPPER_GRIP_WIDTH_RAW = 500
GRIPPER_FORCE_RAW = 200
GRIPPER_TIMEOUT_SEC = 5.0

WINDOW_NAME = "Move To Beverage OpenCV"


@dataclass(frozen=True)
class StableCanResult:
    """여러 프레임에서 안정화한 캔 중심 결과."""

    pixel_x: float
    pixel_y: float
    radius_px: float
    depth_mm: float
    camera_position_mm: np.ndarray


def extract_pose_values(raw_pose, description: str) -> np.ndarray:
    """DSR API 버전에 따른 pose 또는 (pose, solution)을 6개 값으로 통일한다."""
    pose = raw_pose
    if (
        isinstance(pose, (tuple, list))
        and len(pose) > 0
        and isinstance(pose[0], (tuple, list, np.ndarray))
    ):
        pose = pose[0]

    values = np.asarray(pose, dtype=np.float64).reshape(-1)
    if values.size != 6 or not np.all(np.isfinite(values)):
        raise RuntimeError(
            f"{description}가 유효한 6축 pose가 아닙니다: {raw_pose}"
        )
    return values


def validate_base_pose(raw_pose, description: str) -> np.ndarray:
    """BASE 절대 pose가 설정된 작업 범위 안에 있는지 확인한다."""
    values = extract_pose_values(raw_pose, description)
    x, y, z = (float(value) for value in values[:3])

    if not BASE_X_RANGE_MM[0] <= x <= BASE_X_RANGE_MM[1]:
        raise ValueError(
            f"{description} X={x:.1f} mm가 작업 범위 {BASE_X_RANGE_MM} 밖입니다."
        )
    if not BASE_Y_RANGE_MM[0] <= y <= BASE_Y_RANGE_MM[1]:
        raise ValueError(
            f"{description} Y={y:.1f} mm가 작업 범위 {BASE_Y_RANGE_MM} 밖입니다."
        )
    if not BASE_Z_RANGE_MM[0] <= z <= BASE_Z_RANGE_MM[1]:
        raise ValueError(
            f"{description} Z={z:.1f} mm가 작업 범위 {BASE_Z_RANGE_MM} 밖입니다."
        )
    xy_radius = math.hypot(x, y)
    if xy_radius > MAX_BASE_XY_RADIUS_MM:
        raise ValueError(
            f"{description} BASE XY 반경={xy_radius:.1f} mm가 "
            f"한계 {MAX_BASE_XY_RADIUS_MM:.1f} mm를 넘습니다."
        )
    return values


def pose_to_transform(pose: np.ndarray) -> np.ndarray:
    """Doosan posx의 ZYZ 자세를 4x4 변환행렬로 바꾼다."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_euler(
        "ZYZ",
        pose[3:6],
        degrees=True,
    ).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def transform_camera_point_to_base(
    camera_position_mm: np.ndarray,
    flange_pose: np.ndarray,
    flange_to_camera: np.ndarray,
) -> np.ndarray:
    """카메라 기준 XYZ(mm)를 손–카메라 행렬을 사용해 BASE XYZ로 바꾼다."""
    camera_point = np.asarray(
        camera_position_mm,
        dtype=np.float64,
    ).reshape(3)

    # 화면을 180도 돌려 검출했다면 실제 카메라 optical frame으로 되돌린다.
    if ROTATE_CAMERA_180:
        camera_point = np.array(
            [-camera_point[0], -camera_point[1], camera_point[2]],
            dtype=np.float64,
        )

    base_to_flange = pose_to_transform(flange_pose)
    base_to_camera = base_to_flange @ flange_to_camera
    base_homogeneous = base_to_camera @ np.append(camera_point, 1.0)
    result = base_homogeneous[:3]

    if not np.all(np.isfinite(result)):
        raise RuntimeError(f"카메라→BASE 변환 결과가 잘못되었습니다: {result}")
    return result


def command_gripper(
    gripper: RG,
    logger,
    width_raw: int,
    force_raw: int,
    require_grip: bool,
    description: str,
) -> None:
    """RG2 명령 후 완료 상태와 필요 시 물체 파지 상태를 확인한다."""
    logger.info(
        f"{description}: width={width_raw / 10.0:.1f} mm, "
        f"force={force_raw / 10.0:.1f} N"
    )
    gripper.move_gripper(width_val=width_raw, force_val=force_raw)
    time.sleep(0.2)
    deadline = time.monotonic() + GRIPPER_TIMEOUT_SEC

    while time.monotonic() < deadline:
        status = gripper.get_status()
        if len(status) < 7:
            raise RuntimeError(f"RG2 상태 응답 길이가 잘못되었습니다: {status}")
        if any(status[2:7]):
            raise RuntimeError(f"RG2 안전 스위치/회로 오류입니다: {status}")
        if status[0] == 0:
            if require_grip and status[1] != 1:
                raise RuntimeError("RG2가 캔 파지를 감지하지 못했습니다.")
            if not require_grip:
                actual_width = float(gripper.get_width_with_offset())
                target_width = width_raw / 10.0
                if abs(actual_width - target_width) > 3.0:
                    raise RuntimeError(
                        "RG2 열림 폭이 목표에 도달하지 못했습니다: "
                        f"target={target_width:.1f} mm, "
                        f"actual={actual_width:.1f} mm"
                    )
            logger.info(f"{description} 완료")
            return
        time.sleep(0.1)

    raise TimeoutError(
        f"{description}이 {GRIPPER_TIMEOUT_SEC:.1f}초 안에 끝나지 않았습니다."
    )


class BeveragePositionReceiver(Node):
    """YOLO detector가 발행한 첫 번째 유효 BASE 좌표를 저장한다."""

    def __init__(self) -> None:
        super().__init__("move_to_bev_opencv_receiver")
        self.received_position: Optional[Dict[str, float]] = None

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # 과거에 저장된 좌표로 즉시 움직이지 않도록 새 발행만 받는다.
            # TRANSIENT_LOCAL publisher와도 통신 가능한 VOLATILE subscriber다.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.position_subscription = self.create_subscription(
            BeveragePosition,
            BEVERAGE_POSITION_TOPIC,
            self._position_callback,
            qos_profile,
        )

    def _position_callback(self, msg: BeveragePosition) -> None:
        """첫 정상 좌표만 저장하며 콜백 안에서는 로봇을 움직이지 않는다."""
        if self.received_position is not None:
            return

        xyz = (float(msg.base_x), float(msg.base_y), float(msg.base_z))
        confidence = float(msg.confidence)
        if not all(math.isfinite(value) for value in (*xyz, confidence)):
            self.get_logger().warning(
                f"유한수가 아닌 음료 좌표/신뢰도를 무시합니다: {xyz}"
            )
            return
        if confidence < MIN_ROUGH_CONFIDENCE:
            self.get_logger().warning(
                f"신뢰도 {confidence:.3f}가 기준 "
                f"{MIN_ROUGH_CONFIDENCE:.3f}보다 낮아 무시합니다."
            )
            return
        if not (
            BASE_X_RANGE_MM[0] <= xyz[0] <= BASE_X_RANGE_MM[1]
            and BASE_Y_RANGE_MM[0] <= xyz[1] <= BASE_Y_RANGE_MM[1]
            and BASE_Z_RANGE_MM[0] <= xyz[2] <= BASE_Z_RANGE_MM[1]
            and math.hypot(xyz[0], xyz[1]) <= MAX_BASE_XY_RADIUS_MM
        ):
            self.get_logger().warning(
                f"설정된 작업 범위 밖의 음료 좌표를 무시합니다: {xyz}"
            )
            return

        self.received_position = {
            "class_name": str(msg.class_name),
            "confidence": confidence,
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
        }
        self.get_logger().info(
            "거친 음료 좌표 수신: "
            f"class={msg.class_name}, confidence={msg.confidence:.3f}, "
            f"BASE XYZ=({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) mm"
        )


class CanTopVisionNode(Node):
    """수직 화면의 원과 Depth를 여러 프레임에서 안정화한다."""

    def __init__(self) -> None:
        super().__init__("move_to_bev_opencv_vision")
        self.bridge = CvBridge()
        self.color_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.color_stamp: Optional[float] = None
        self.depth_stamp: Optional[float] = None
        self.intrinsics: Optional[Dict[str, float]] = None
        self.frame_generation = 0
        self.last_processed_generation = -1
        self.samples: List[CircleObservation] = []
        self.result: Optional[StableCanResult] = None
        self.abort_requested = False
        self.window_enabled = True

        self.color_subscription = self.create_subscription(
            Image,
            COLOR_TOPIC,
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

    def _color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.color_frame = np.asarray(frame)
            self.color_stamp = message_stamp_sec(msg)
            self.frame_generation += 1
        except Exception as error:
            self.get_logger().error(f"RGB 변환 실패: {error}")

    def _depth_callback(self, msg: Image) -> None:
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

    def _camera_info_callback(self, msg: CameraInfo) -> None:
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

    def start_collection(self) -> None:
        """현재까지의 프레임을 버리고 정렬 이후 새 프레임만 수집한다."""
        self.samples.clear()
        self.result = None
        self.abort_requested = False
        self.last_processed_generation = self.frame_generation
        self.get_logger().info(
            f"캔 원 중심 {VISION_REQUIRED_SAMPLES}개 안정화 수집을 시작합니다."
        )

    def _pixel_to_camera(
        self,
        pixel_x: float,
        pixel_y: float,
        depth_mm: float,
    ) -> np.ndarray:
        """정렬 Depth 픽셀을 RealSense optical-frame XYZ(mm)로 역투영한다."""
        return np.array(
            [
                (pixel_x - self.intrinsics["ppx"])
                * depth_mm
                / self.intrinsics["fx"],
                (pixel_y - self.intrinsics["ppy"])
                * depth_mm
                / self.intrinsics["fy"],
                depth_mm,
            ],
            dtype=np.float64,
        )

    def _try_make_stable_result(self) -> Optional[StableCanResult]:
        if len(self.samples) < VISION_REQUIRED_SAMPLES:
            return None

        recent = self.samples[-VISION_REQUIRED_SAMPLES:]
        values = np.asarray(
            [
                [sample.x, sample.y, sample.radius, sample.depth_mm]
                for sample in recent
            ],
            dtype=np.float64,
        )
        median = np.median(values, axis=0)
        center_spread = np.max(
            np.linalg.norm(values[:, :2] - median[:2], axis=1)
        )
        depth_range = float(np.ptp(values[:, 3]))
        median_radius = float(median[2])
        radius_deviation = float(
            np.max(np.abs(values[:, 2] - median_radius))
            / max(1.0, median_radius)
        )

        if (
            center_spread > VISION_CENTER_MAX_SPREAD_PX
            or depth_range > VISION_DEPTH_MAX_RANGE_MM
            or radius_deviation > VISION_RADIUS_MAX_RELATIVE_DEVIATION
        ):
            self.get_logger().warning(
                "원 중심이 아직 불안정합니다: "
                f"center_spread={center_spread:.1f}px, "
                f"depth_range={depth_range:.1f}mm, "
                f"radius_deviation={radius_deviation:.2f}"
            )
            return None

        camera_position = self._pixel_to_camera(
            float(median[0]),
            float(median[1]),
            float(median[3]),
        )
        return StableCanResult(
            pixel_x=float(median[0]),
            pixel_y=float(median[1]),
            radius_px=median_radius,
            depth_mm=float(median[3]),
            camera_position_mm=camera_position,
        )

    def process_latest_frame(self) -> None:
        """가장 최근 RGB/Depth 한 쌍을 검출하고 안정화 샘플에 추가한다."""
        if (
            self.result is not None
            or self.color_frame is None
            or self.depth_frame is None
            or self.intrinsics is None
            or self.frame_generation == self.last_processed_generation
        ):
            return

        self.last_processed_generation = self.frame_generation
        color_frame = self.color_frame.copy()
        depth_frame = self.depth_frame.copy()

        if color_frame.shape[:2] != depth_frame.shape[:2]:
            self.get_logger().warning(
                "RGB와 aligned Depth 해상도가 달라 프레임을 건너뜁니다: "
                f"RGB={color_frame.shape[:2]}, Depth={depth_frame.shape[:2]}"
            )
            return

        if (
            self.color_stamp is None
            or self.depth_stamp is None
            or abs(self.color_stamp - self.depth_stamp)
            > RGB_DEPTH_MAX_STAMP_DIFF_SEC
        ):
            return

        if ROTATE_CAMERA_180:
            color_frame = cv2.rotate(color_frame, cv2.ROTATE_180)
            depth_frame = cv2.rotate(depth_frame, cv2.ROTATE_180)

        selected, candidates, roi_info = detect_can_top_circle(
            color_frame,
            depth_frame,
            self.intrinsics,
        )
        if selected is not None and selected.depth_mm is not None:
            self.samples.append(selected)
            self.samples = self.samples[-VISION_REQUIRED_SAMPLES:]
            self.get_logger().info(
                f"원 중심 수집 {len(self.samples)}/{VISION_REQUIRED_SAMPLES}: "
                f"pixel=({selected.x}, {selected.y}), "
                f"depth={selected.depth_mm:.1f} mm, r={selected.radius}px"
            )
            self.result = self._try_make_stable_result()

        if self.window_enabled:
            try:
                output = draw_detection_overlay(
                    color_frame,
                    selected,
                    candidates,
                    roi_info,
                )
                cv2.imshow(WINDOW_NAME, output)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    self.abort_requested = True
            except cv2.error as error:
                self.get_logger().warning(
                    f"OpenCV 창을 열 수 없어 화면 표시를 끕니다: {error}"
                )
                self.window_enabled = False

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def load_flange_camera_transform() -> np.ndarray:
    """설치된 패키지에서 T_flange_camera.npy를 읽고 검증한다."""
    transform_path = (
        Path(get_package_share_directory("hotel_vision"))
        / "calibration"
        / "T_flange_camera.npy"
    )
    if not transform_path.exists():
        raise FileNotFoundError(
            f"손–카메라 보정행렬이 없습니다: {transform_path}"
        )

    transform = np.asarray(np.load(str(transform_path)), dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(
            "T_flange_camera.npy는 유효한 4x4 행렬이어야 합니다: "
            f"shape={transform.shape}"
        )
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError("T_flange_camera.npy의 마지막 행이 올바르지 않습니다.")
    rotation = transform[:3, :3]
    if not (
        np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-3)
        and np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-3)
    ):
        raise ValueError("T_flange_camera.npy의 회전행렬이 올바르지 않습니다.")
    return transform


def checked_movel(
    movel,
    target,
    velocity: float,
    acceleration: float,
    reference,
    mode,
    description: str,
) -> None:
    """movel을 실행하고 Doosan 반환값이 0인지 확인한다."""
    result = movel(
        target,
        vel=velocity,
        acc=acceleration,
        ref=reference,
        mod=mode,
    )
    if result != 0:
        raise RuntimeError(f"{description} 실패, 반환값: {result}")


def main(args=None) -> None:
    rclpy.init(args=args)
    robot_node = None
    receiver_node = None
    vision_node = None
    gripper = None
    holding_object = False

    try:
        # DSR_ROBOT2 import 전에 반드시 DR_init에 생성된 노드를 넣어야 한다.
        robot_node = rclpy.create_node(
            "move_to_bev_opencv_robot",
            namespace=ROBOT_ID,
        )
        DR_init.__dsr__node = robot_node

        from DSR_ROBOT2 import (
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            get_current_posx,
            get_current_tool_flange_posx,
            movej,
            movel,
            posj,
            posx,
        )

        place_approach = None
        place_release = None
        place_min_transport_z = None
        if ENABLE_TRAY_PLACE:
            place_approach = validate_base_pose(
                PLACE_APPROACH_POSE,
                "트레이 접근 pose",
            )
            if np.allclose(place_approach, 0.0, atol=1.0e-9):
                raise ValueError(
                    "PLACE_APPROACH_POSE가 모두 0인 placeholder입니다."
                )
            if not math.isfinite(PLACE_RELEASE_Z_MM):
                raise ValueError("PLACE_RELEASE_Z_MM은 유한한 값이어야 합니다.")
            if PLACE_RELEASE_Z_MM >= float(place_approach[2]):
                raise ValueError(
                    "PLACE_RELEASE_Z_MM은 PLACE_APPROACH_POSE의 Z보다 "
                    "작아야 합니다."
                )
            if (
                not math.isfinite(PLACE_RETREAT_CLEARANCE_MM)
                or PLACE_RETREAT_CLEARANCE_MM <= 0.0
            ):
                raise ValueError(
                    "PLACE_RETREAT_CLEARANCE_MM은 유한한 양수여야 합니다."
                )

            place_release = place_approach.copy()
            place_release[2] = PLACE_RELEASE_Z_MM
            validate_base_pose(place_release, "트레이 release pose")

            # 이 값이 잘못되어 있으면 캔을 잡은 뒤가 아니라 로봇을 움직이기
            # 전에 중단하도록 미리 검사한다.
            place_min_transport_z = max(
                float(place_approach[2]),
                float(place_release[2]) + PLACE_RETREAT_CLEARANCE_MM,
            )
            configured_transport = place_approach.copy()
            configured_transport[2] = place_min_transport_z
            validate_base_pose(configured_transport, "트레이 안전 운반 pose")

        # 변환행렬이 없으면 로봇을 움직이기 전에 즉시 중단한다.
        flange_to_camera = load_flange_camera_transform()

        robot_node.get_logger().info(
            f"초기 관절 자세로 이동합니다: {INITIAL_JOINT_POSE}"
        )
        movej_result = movej(
            posj(INITIAL_JOINT_POSE),
            vel=JOINT_VELOCITY,
            acc=JOINT_ACCELERATION,
        )
        if movej_result != 0:
            raise RuntimeError(f"초기 movej 실패, 반환값: {movej_result}")

        gripper = RG("rg2", GRIPPER_IP, GRIPPER_PORT)
        command_gripper(
            gripper,
            robot_node.get_logger(),
            GRIPPER_OPEN_WIDTH_RAW,
            GRIPPER_FORCE_RAW,
            require_grip=False,
            description="RG2 열기",
        )

        # 초기 자세 도착 전에 detector가 만든 좌표는 사용하지 않는다.
        # 여기서 VOLATILE 구독을 새로 만들어 이후에 발행되는 좌표만 받는다.
        receiver_node = BeveragePositionReceiver()
        vision_node = CanTopVisionNode()

        receiver_node.get_logger().info(
            f"{BEVERAGE_POSITION_TOPIC}의 새 좌표를 기다립니다. "
            "detector가 이미 발행을 끝냈다면 Q를 눌러 다시 수집하세요."
        )
        while rclpy.ok() and receiver_node.received_position is None:
            rclpy.spin_once(receiver_node, timeout_sec=0.05)
            rclpy.spin_once(vision_node, timeout_sec=0.0)

        if not rclpy.ok():
            return
        received = receiver_node.received_position

        # 카메라가 준비되지 않았다면 캔 쪽으로 이동하기 전에 중단한다.
        camera_deadline = time.monotonic() + CAMERA_READY_TIMEOUT_SEC
        while (
            rclpy.ok()
            and (
                vision_node.color_frame is None
                or vision_node.depth_frame is None
                or vision_node.intrinsics is None
            )
            and time.monotonic() < camera_deadline
        ):
            rclpy.spin_once(vision_node, timeout_sec=0.05)

        if (
            vision_node.color_frame is None
            or vision_node.depth_frame is None
            or vision_node.intrinsics is None
        ):
            raise TimeoutError(
                f"{CAMERA_READY_TIMEOUT_SEC:.1f}초 안에 RGB/Depth/CameraInfo가 "
                "모두 준비되지 않았습니다."
            )

        # 1) 대각선으로 낮게 가는 경로를 피하기 위해 현재 위치에서 먼저
        # 안전 높이까지 BASE +Z로 올린다.
        initial_tcp = extract_pose_values(
            get_current_posx(ref=DR_BASE),
            "초기 TCP pose",
        )
        safe_z = max(
            float(initial_tcp[2]),
            received["z"] + SAFE_CLEARANCE_ABOVE_CAN_MM,
        )
        if safe_z > BASE_Z_RANGE_MM[1]:
            raise RuntimeError(
                f"계산된 안전 높이 {safe_z:.1f} mm가 Z 작업 범위를 넘습니다."
            )

        if safe_z - initial_tcp[2] > 0.5:
            safe_lift_values = [
                float(initial_tcp[0]),
                float(initial_tcp[1]),
                safe_z,
                *[float(value) for value in initial_tcp[3:6]],
            ]
            robot_node.get_logger().info(
                f"현재 XY에서 안전 높이로 상승: {safe_lift_values}"
            )
            checked_movel(
                movel,
                posx(safe_lift_values),
                LINEAR_VELOCITY,
                LINEAR_ACCELERATION,
                DR_BASE,
                DR_MV_MOD_ABS,
                "안전 높이 상승 movel",
            )

        # 2) 안전 높이에서 TOOL이 아래를 보는 자세로 정렬한다.
        vertical_pose_values = [
            float(initial_tcp[0]),
            float(initial_tcp[1]),
            safe_z,
            *VERTICAL_TOOL_ABC,
        ]
        robot_node.get_logger().info(
            f"안전 높이에서 수직 정렬: {vertical_pose_values}"
        )
        checked_movel(
            movel,
            posx(vertical_pose_values),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "수직 정렬 movel",
        )

        # 3) 안전 높이를 유지한 채 캔의 거친 X/Y 위로 수평 이동한다.
        safe_above_can_values = [
            received["x"],
            received["y"],
            safe_z,
            *VERTICAL_TOOL_ABC,
        ]
        robot_node.get_logger().info(
            f"안전 높이에서 캔 위로 XY 이동: {safe_above_can_values}"
        )
        checked_movel(
            movel,
            posx(safe_above_can_values),
            LINEAR_VELOCITY,
            LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "안전 높이 XY 접근 movel",
        )

        # 4) 수직 자세를 유지하고 거친 캔 좌표에서 설정한 +Z까지 내려간다.
        rough_pose_values = [
            received["x"],
            received["y"],
            received["z"] + ROUGH_APPROACH_Z_OFFSET_MM,
            *VERTICAL_TOOL_ABC,
        ]
        robot_node.get_logger().info(f"OpenCV 촬영 접근 pose: {rough_pose_values}")
        checked_movel(
            movel,
            posx(rough_pose_values),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "OpenCV 촬영 높이 접근 movel",
        )

        # 카메라가 흔들리지 않도록 기다리며 최신 프레임을 받는다.
        settle_deadline = time.monotonic() + CAMERA_SETTLE_SEC
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(vision_node, timeout_sec=0.05)

        # 5) ROI 안 캔 원 중심과 Depth가 안정될 때까지 새 프레임을 수집한다.
        vision_node.start_collection()
        vision_deadline = time.monotonic() + VISION_TIMEOUT_SEC
        while (
            rclpy.ok()
            and vision_node.result is None
            and not vision_node.abort_requested
            and time.monotonic() < vision_deadline
        ):
            rclpy.spin_once(vision_node, timeout_sec=0.05)
            vision_node.process_latest_frame()

        if vision_node.abort_requested:
            raise RuntimeError("ESC/Q 입력으로 OpenCV 정밀 검출을 취소했습니다.")
        if vision_node.result is None:
            raise TimeoutError(
                f"{VISION_TIMEOUT_SEC:.1f}초 안에 안정적인 캔 원/Depth를 "
                "얻지 못했습니다. ROI와 Hough 설정을 확인하세요."
            )

        stable = vision_node.result
        robot_node.get_logger().info(
            "안정화된 캔 원: "
            f"pixel=({stable.pixel_x:.1f}, {stable.pixel_y:.1f}), "
            f"depth={stable.depth_mm:.1f} mm, r={stable.radius_px:.1f}px"
        )

        # 6) 카메라 좌표를 BASE로 변환한다. 로봇이 정지한 상태에서 얻은
        # flange pose를 사용하므로 영상 촬영 자세와 변환 자세가 일치한다.
        flange_pose = extract_pose_values(
            get_current_tool_flange_posx(ref=DR_BASE),
            "현재 Tool Flange pose",
        )
        refined_base = transform_camera_point_to_base(
            stable.camera_position_mm,
            flange_pose,
            flange_to_camera,
        )
        current_tcp = extract_pose_values(
            get_current_posx(ref=DR_BASE),
            "현재 TCP pose",
        )

        xy_correction = float(
            np.hypot(
                refined_base[0] - current_tcp[0],
                refined_base[1] - current_tcp[1],
            )
        )
        vertical_gap = float(current_tcp[2] - refined_base[2])
        robot_node.get_logger().info(
            "OpenCV 정밀 캔 중심 (BASE): "
            f"XYZ=({refined_base[0]:.2f}, {refined_base[1]:.2f}, "
            f"{refined_base[2]:.2f}) mm, "
            f"XY 보정={xy_correction:.2f} mm, "
            f"현재 TCP와 높이 차={vertical_gap:.2f} mm"
        )

        if xy_correction > MAX_XY_CORRECTION_MM:
            raise RuntimeError(
                f"XY 보정량 {xy_correction:.1f} mm가 안전 한계 "
                f"{MAX_XY_CORRECTION_MM:.1f} mm를 넘었습니다."
            )
        if not (
            MIN_CAN_VERTICAL_GAP_MM
            <= vertical_gap
            <= MAX_CAN_VERTICAL_GAP_MM
        ):
            raise RuntimeError(
                f"캔 윗면 높이 차 {vertical_gap:.1f} mm가 허용 범위 "
                f"[{MIN_CAN_VERTICAL_GAP_MM:.1f}, "
                f"{MAX_CAN_VERTICAL_GAP_MM:.1f}] mm 밖입니다."
            )

        # 7) 현재 높이와 수직 ABC는 유지하고 BASE X/Y만 캔 중심에 맞춘다.
        centered_pose_values = [
            float(refined_base[0]),
            float(refined_base[1]),
            float(current_tcp[2]),
            *VERTICAL_TOOL_ABC,
        ]
        robot_node.get_logger().info(
            f"캔 중심 위로 XY 보정 이동: {centered_pose_values}"
        )
        checked_movel(
            movel,
            posx(centered_pose_values),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "캔 중심 XY 보정 movel",
        )

        if not ENABLE_FINAL_DESCENT_AND_GRIP:
            robot_node.get_logger().warning(
                "안전 설정으로 최종 하강/파지를 생략했습니다. 로그와 위치를 "
                "확인한 뒤 ENABLE_FINAL_DESCENT_AND_GRIP=True로 바꾸세요."
            )
            return

        grasp_target_z = (
            float(refined_base[2]) + GRASP_TCP_Z_OFFSET_FROM_CAN_TOP_MM
        )
        final_descent = float(current_tcp[2] - grasp_target_z)
        if not MIN_FINAL_DESCENT_MM <= final_descent <= MAX_FINAL_DESCENT_MM:
            raise RuntimeError(
                f"검출 높이로 계산한 하강량 {final_descent:.1f} mm가 허용 "
                f"범위 [{MIN_FINAL_DESCENT_MM:.1f}, "
                f"{MAX_FINAL_DESCENT_MM:.1f}] mm 밖입니다. "
                "TCP 파지 중심 오프셋을 확인하세요."
            )

        # 8) 검출 캔 윗면 기준 목표 Z까지 하강하고 낮은 힘으로 파지한다.
        robot_node.get_logger().info(
            f"BASE -Z {final_descent:.1f} mm로 천천히 하강합니다. "
            f"목표 TCP Z={grasp_target_z:.1f} mm"
        )
        checked_movel(
            movel,
            posx([0.0, 0.0, -final_descent, 0.0, 0.0, 0.0]),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_REL,
            "최종 BASE -Z 하강 movel",
        )
        # 명령 전송 뒤 통신/상태 확인이 실패해도 실제로 캔을 잡았을 수 있으므로
        # 이 시점부터 보수적으로 holding 상태로 취급한다.
        holding_object = True
        try:
            command_gripper(
                gripper,
                robot_node.get_logger(),
                GRIPPER_GRIP_WIDTH_RAW,
                GRIPPER_FORCE_RAW,
                require_grip=True,
                description="캔 파지",
            )
        except Exception:
            robot_node.get_logger().error(
                "파지 실패: 낮은 위치에 머물지 않도록 같은 거리만큼 상승합니다."
            )
            retreat_result = movel(
                posx([0.0, 0.0, final_descent, 0.0, 0.0, 0.0]),
                vel=FINE_LINEAR_VELOCITY,
                acc=FINE_LINEAR_ACCELERATION,
                ref=DR_BASE,
                mod=DR_MV_MOD_REL,
            )
            if retreat_result != 0:
                robot_node.get_logger().error(
                    f"파지 실패 후 상승도 실패했습니다: {retreat_result}"
                )
            raise

        # 파지 후 캔을 끌지 않도록 방금 내려온 경로로 다시 상승한다.
        checked_movel(
            movel,
            posx([0.0, 0.0, final_descent, 0.0, 0.0, 0.0]),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_REL,
            "파지 후 BASE +Z 상승 movel",
        )
        robot_node.get_logger().info(
            "OpenCV 기반 캔 수직 파지 및 접근 높이 복귀를 완료했습니다."
        )

        if not ENABLE_TRAY_PLACE:
            robot_node.get_logger().warning(
                "ENABLE_TRAY_PLACE=False이므로 캔을 든 상태에서 배치 작업을 "
                "생략합니다. 로봇/그리퍼를 수동 조작하기 전에 상태를 "
                "확인하세요."
            )
            return

        if (
            place_approach is None
            or place_release is None
            or place_min_transport_z is None
        ):
            raise RuntimeError("트레이 배치 설정이 초기화되지 않았습니다.")

        # 9) 캔을 든 상태에서는 곧바로 대각선 이동하지 않는다. 현재 XY에서
        # 먼저 안전 운반 높이까지 수직 상승한 뒤 트레이 위로 수평 이동한다.
        lifted_tcp = extract_pose_values(
            get_current_posx(ref=DR_BASE),
            "파지 후 상승 TCP pose",
        )
        transport_z = max(
            float(lifted_tcp[2]),
            place_min_transport_z,
        )
        if transport_z > BASE_Z_RANGE_MM[1]:
            raise RuntimeError(
                f"계산된 트레이 운반 높이 {transport_z:.1f} mm가 Z 작업 "
                f"범위 {BASE_Z_RANGE_MM} 밖입니다."
            )

        pick_transport_values = [
            float(lifted_tcp[0]),
            float(lifted_tcp[1]),
            transport_z,
            *[float(value) for value in lifted_tcp[3:6]],
        ]
        validate_base_pose(pick_transport_values, "파지 위치 안전 운반 pose")
        if transport_z - float(lifted_tcp[2]) > 0.5:
            robot_node.get_logger().info(
                "캔을 든 채 현재 XY에서 안전 운반 높이로 상승: "
                f"Z={transport_z:.1f} mm"
            )
            checked_movel(
                movel,
                posx(pick_transport_values),
                LINEAR_VELOCITY,
                LINEAR_ACCELERATION,
                DR_BASE,
                DR_MV_MOD_ABS,
                "파지 후 안전 운반 높이 상승 movel",
            )

        # 먼저 파지 자세를 그대로 유지한 채 트레이 X/Y 위로 수평 이동한다.
        tray_above_vertical_values = [
            float(place_approach[0]),
            float(place_approach[1]),
            transport_z,
            *[float(value) for value in lifted_tcp[3:6]],
        ]
        validate_base_pose(
            tray_above_vertical_values,
            "수직 파지 자세의 트레이 안전 운반 pose",
        )
        robot_node.get_logger().info(
            "수직 파지 자세를 유지하고 트레이 위로 이동: "
            f"{tray_above_vertical_values}"
        )
        checked_movel(
            movel,
            posx(tray_above_vertical_values),
            LINEAR_VELOCITY,
            LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "트레이 위 안전 운반 movel",
        )

        # 트레이 위 안전 높이에 도착한 뒤에만 배치용 ABC로 자세를 바꾼다.
        place_transport_values = [
            float(place_approach[0]),
            float(place_approach[1]),
            transport_z,
            *[float(value) for value in place_approach[3:6]],
        ]
        validate_base_pose(place_transport_values, "트레이 배치 자세 pose")
        robot_node.get_logger().info(
            f"트레이 위 안전 높이에서 배치 자세로 정렬: {place_transport_values}"
        )
        checked_movel(
            movel,
            posx(place_transport_values),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "트레이 위 배치 자세 정렬 movel",
        )

        # 10) 트레이 접근점과 release Z까지 차례로 수직 하강한다.
        robot_node.get_logger().info(
            f"트레이 접근 pose로 하강: {place_approach.tolist()}"
        )
        checked_movel(
            movel,
            posx(place_approach.tolist()),
            LINEAR_VELOCITY,
            LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "트레이 접근 movel",
        )
        robot_node.get_logger().info(
            f"트레이 release Z로 천천히 하강: {place_release.tolist()}"
        )
        checked_movel(
            movel,
            posx(place_release.tolist()),
            FINE_LINEAR_VELOCITY,
            FINE_LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "트레이 release 하강 movel",
        )

        # 11) 목표 높이에서 캔을 놓고 같은 경로로 안전 높이까지 후퇴한다.
        command_gripper(
            gripper,
            robot_node.get_logger(),
            GRIPPER_OPEN_WIDTH_RAW,
            GRIPPER_FORCE_RAW,
            require_grip=False,
            description="트레이에 캔 놓기",
        )
        holding_object = False
        checked_movel(
            movel,
            posx(place_transport_values),
            LINEAR_VELOCITY,
            LINEAR_ACCELERATION,
            DR_BASE,
            DR_MV_MOD_ABS,
            "배치 후 안전 높이 후퇴 movel",
        )

        if RETURN_TO_INITIAL_POSE_AFTER_PLACE:
            robot_node.get_logger().info("배치 후 초기 관절 자세로 복귀합니다.")
            return_home_result = movej(
                posj(INITIAL_JOINT_POSE),
                vel=JOINT_VELOCITY,
                acc=JOINT_ACCELERATION,
            )
            if return_home_result != 0:
                raise RuntimeError(
                    "배치 후 초기 movej 실패, "
                    f"반환값: {return_home_result}"
                )

        robot_node.get_logger().info("캔 트레이 배치를 완료했습니다.")

    except KeyboardInterrupt:
        if robot_node is not None:
            robot_node.get_logger().info("사용자 요청으로 종료합니다.")
            if holding_object:
                robot_node.get_logger().warning(
                    "캔을 들고 있을 수 있어 그리퍼를 자동으로 열지 않습니다."
                )
    except Exception as error:
        if robot_node is not None:
            robot_node.get_logger().error(f"move_to_bev_opencv 오류: {error}")
            if holding_object:
                robot_node.get_logger().error(
                    "캔을 들고 있을 수 있어 그리퍼를 자동으로 열지 않습니다. "
                    "로봇 상태와 주변 안전을 먼저 확인하세요."
                )
        else:
            print(f"move_to_bev_opencv 노드 생성 실패: {error}")
        raise
    finally:
        if gripper is not None:
            gripper.close_connection()
        if vision_node is not None:
            vision_node.destroy_node()
        if receiver_node is not None:
            receiver_node.destroy_node()
        if robot_node is not None:
            robot_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
