import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from checkin_interfaces.msg import BeveragePosition
from rclpy.node import Node 
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# beverage_detector_node
class BeverageDetectorNode(Node):
    """음료를 검출하고 로봇 베이스 기준 좌표를 발행하는 노드."""

    TARGET_CLASS_NAMES = {
        "coffee",
        # "cola",
        # "tea",
        # "sikhye",
    }

    # 카메라가 거꾸로 장착된 경우 RGB와 Depth를 함께 180도 회전한다.
    ROTATE_CAMERA_180 = False

    CONFIDENCE_THRESHOLD = 0.50
    REQUIRED_CONSECUTIVE_FRAMES = 1

    # 0.2초 간격으로 유효 좌표 5개를 수집한다.
    SAMPLE_INTERVAL_SEC = 0.2
    REQUIRED_VALID_SAMPLES = 5

    # 중앙값에서 이 거리(mm)보다 크게 벗어난 샘플은 이상치로 제거한다.
    OUTLIER_DISTANCE_THRESHOLD_MM = 10.0

    # 바운딩 박스의 위쪽에서 몇 % 지점을 대표점으로 사용할지 결정한다.
    # 0.5는 정중앙, 0.6은 중앙보다 약간 아래쪽이다.
    GRASP_POINT_Y_RATIO = 0.60

    # 중심 주변 Depth 탐색 반경이다.
    # radius=4이면 9x9 영역을 검사한다.
    DEPTH_SAMPLE_RADIUS = 4

    # 비정상적인 깊이를 제거하기 위한 범위(mm)
    MIN_DEPTH_MM = 100.0
    MAX_DEPTH_MM = 2000.0

    def __init__(self):
        super().__init__("beverage_detector_node")

        self.completed = False
        self.consecutive_frames = 0
        self.last_target_class: Optional[str] = None
        self.collecting = True
        self.samples = []
        self.last_sample_time = 0.0

        package_share = Path(
            get_package_share_directory("hotel_vision")
        )

        self.model_path = package_share / "models" / "beverage_best.pt"
        self.transform_path = (
            package_share
            / "calibration"
            / "T_flange_camera.npy"
        )

        self._validate_resource_files()

        self.get_logger().info(
            f"음료 모델 로딩: {self.model_path}"
        )
        self.model = YOLO(str(self.model_path))

        self.flange_camera = np.load(
            str(self.transform_path)
        )

        if self.flange_camera.shape != (4, 4):
            raise ValueError(
                "T_flange_camera.npy는 4x4 행렬이어야 합니다. "
                f"현재 shape: {self.flange_camera.shape}"
            )

        # RealSense ROS 토픽을 직접 구독한다.
        self.bridge = CvBridge()
        self.color_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.intrinsics = None



        self.color_subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10,
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10,
        )

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.position_publisher = self.create_publisher(
            BeveragePosition,
            "/checkin/beverage_position",
            qos_profile,
        )

        self.timer = self.create_timer(
            0.1,
            self.detect_and_publish,
        )

        self.get_logger().info(
            "음료 위치 인식을 시작합니다."
        )
        self.get_logger().info(
            f"검출 조건: confidence >= "
            f"{self.CONFIDENCE_THRESHOLD}, "
            f"{self.REQUIRED_CONSECUTIVE_FRAMES}회 연속 검출"
        )
        self.get_logger().info(
            f"{self.SAMPLE_INTERVAL_SEC:.1f}초 간격으로 "
            f"{self.REQUIRED_VALID_SAMPLES}개 좌표를 수집한 뒤 "
            "이상치를 제거하고 평균 좌표를 1회 발행합니다."
        )
        self.get_logger().info(
            "Q: 새 5회 수집 시작 / ESC: 종료"
        )

    def _validate_resource_files(self) -> None:
        if not self.model_path.exists():
            raise FileNotFoundError(
                "음료 인식 모델이 없습니다.\n"
                f"다음 경로에 best.pt를 넣으세요:\n"
                f"{self.model_path}"
            )

        if not self.transform_path.exists():
            raise FileNotFoundError(
                "손목-카메라 변환행렬이 없습니다.\n"
                f"다음 경로에 T_flange_camera.npy를 넣으세요:\n"
                f"{self.transform_path}"
            )

    def color_callback(self, msg: Image) -> None:
        """RealSense RGB 토픽을 OpenCV BGR 영상으로 변환한다."""

        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"RGB 영상 변환 실패: {error}"
            )

    def depth_callback(self, msg: Image) -> None:
        """RGB에 정렬된 Depth 토픽을 원본 단위로 변환한다."""

        try:
            depth_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )

            # 일반적인 RealSense 16UC1 Depth는 이미 mm 단위이다.
            # 32FC1로 들어오는 경우 m 단위를 mm로 변환한다.
            if depth_frame.dtype == np.float32:
                depth_frame = depth_frame * 1000.0

            self.depth_frame = np.asarray(depth_frame)

        except Exception as error:
            self.get_logger().error(
                f"Depth 영상 변환 실패: {error}"
            )

    def camera_info_callback(
        self,
        msg: CameraInfo,
    ) -> None:
        """RGB 카메라 내부 파라미터를 저장한다."""

        ppx = float(msg.k[2])
        ppy = float(msg.k[5])

        # 영상을 180도 회전하면 주점도 회전된 영상 좌표에 맞게 바꾼다.
        if self.ROTATE_CAMERA_180:
            ppx = float(msg.width - 1) - ppx
            ppy = float(msg.height - 1) - ppy

        self.intrinsics = {
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "ppx": ppx,
            "ppy": ppy,
        }

    def detect_and_publish(self) -> None:
        if self.completed:
            return

        color_frame = self.color_frame
        depth_frame = self.depth_frame

        if self.intrinsics is None:
            self.get_logger().warning(
                "CameraInfo를 아직 수신하지 못했습니다."
            )
            return

        if color_frame is None:
            self.get_logger().warning(
                "RGB 프레임을 가져오지 못했습니다."
            )
            return

        if depth_frame is None or np.all(depth_frame == 0):
            self.get_logger().warning(
                "Depth 프레임을 가져오지 못했습니다."
            )
            return

        if color_frame.shape[:2] != depth_frame.shape[:2]:
            self.get_logger().warning(
                "RGB와 정렬 Depth 해상도가 다릅니다: "
                f"RGB={color_frame.shape[:2]}, "
                f"Depth={depth_frame.shape[:2]}"
            )
            return

        # 같은 픽셀 위치의 RGB와 Depth 대응을 유지하기 위해
        # 두 영상을 반드시 함께 회전한다.
        if self.ROTATE_CAMERA_180:
            color_frame = cv2.rotate(
                color_frame,
                cv2.ROTATE_180,
            )
            depth_frame = cv2.rotate(
                depth_frame,
                cv2.ROTATE_180,
            )

        results = self.model.predict(
            source=color_frame,
            conf=self.CONFIDENCE_THRESHOLD,
            imgsz=640,
            verbose=False,
            device="cpu",
        )

        target = self.select_best_detection(results)

        annotated_frame = color_frame.copy()

        if target is None:
            self.reset_detection_counter()
        else:
            (
                class_name,
                confidence,
                x1,
                y1,
                x2,
                y2,
            ) = target

            target_u, target_v = self.select_target_pixel(
                x1,
                y1,
                x2,
                y2,
            )

            cv2.rectangle(
                annotated_frame,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            cv2.circle(
                annotated_frame,
                (target_u, target_v),
                6,
                (0, 0, 255),
                -1,
            )

            label = f"{class_name} {confidence:.2f}"
            cv2.putText(
                annotated_frame,
                label,
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            self.update_detection_counter(class_name)

            # self.get_logger().info(
            #     f"음료 검출: {class_name}, "
            #     f"confidence={confidence:.3f}, "
            #     f"연속={self.consecutive_frames}/"
            #     f"{self.REQUIRED_CONSECUTIVE_FRAMES}"
            # )

            if (
                self.collecting
                and self.consecutive_frames >= self.REQUIRED_CONSECUTIVE_FRAMES
            ):
                current_time = time.monotonic()
                if current_time - self.last_sample_time >= self.SAMPLE_INTERVAL_SEC:
                    depth_mm = self.get_stable_depth(target_u, target_v, depth_frame)
                    if depth_mm is None:
                        self.get_logger().warning(
                            "대표 지점에서 유효한 Depth를 얻지 못했습니다."
                        )
                    else:
                        camera_position = self.pixel_to_camera_position(
                            target_u, target_v, depth_mm
                        )
                        base_position = self.transform_camera_to_base(camera_position)
                        self.samples.append(
                            (base_position, float(confidence), class_name)
                        )
                        self.last_sample_time = current_time
                        self.get_logger().info(
                            f"좌표 수집 {len(self.samples)}/"
                            f"{self.REQUIRED_VALID_SAMPLES}: "
                            f"base=({base_position[0]:.2f}, "
                            f"{base_position[1]:.2f}, "
                            f"{base_position[2]:.2f}) mm"
                        )

                        if len(self.samples) >= self.REQUIRED_VALID_SAMPLES:
                            self.finish_sampling()

        cv2.imshow(
            "Beverage Detector",
            annotated_frame,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.start_sampling()
        elif key == 27:
            self.get_logger().info("ESC키로 종료합니다.")
            rclpy.shutdown()

    def start_sampling(self) -> None:
        """현재 샘플을 버리고 새 5회 수집을 시작한다."""
        self.samples.clear()
        self.collecting = True
        self.last_sample_time = 0.0
        self.get_logger().info("새 좌표 수집을 시작합니다.")

    def finish_sampling(self) -> None:
        """이상치를 제거하고 남은 좌표의 평균을 1회 발행한다."""
        positions = np.asarray([sample[0] for sample in self.samples], dtype=np.float64)
        median_position = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median_position, axis=1)
        keep_mask = distances <= self.OUTLIER_DISTANCE_THRESHOLD_MM

        # 너무 많은 샘플이 제거되면 중앙값에서 가장 가까운 3개를 사용한다.
        if np.count_nonzero(keep_mask) < 3:
            keep_mask = np.zeros(len(positions), dtype=bool)
            keep_mask[np.argsort(distances)[:3]] = True

        filtered_positions = positions[keep_mask]
        final_position = np.mean(filtered_positions, axis=0)
        confidences = np.asarray(
            [sample[1] for sample in self.samples], dtype=np.float64
        )
        final_confidence = float(np.mean(confidences[keep_mask]))
        class_name = self.samples[0][2]
        removed_count = len(positions) - len(filtered_positions)

        self.publish_position(class_name, final_confidence, final_position)
        self.collecting = False
        self.get_logger().info(
            f"수집 완료: {len(filtered_positions)}개 사용, "
            f"{removed_count}개 이상치 제거. Q를 누르면 다시 수집합니다."
        )

        self.get_logger().info(
            "최종 결과 좌표: "
            f"x={final_position[0]:.2f}, "
            f"y={final_position[1]:.2f}, "
            f"z={final_position[2]:.2f} mm"
        )

    def select_best_detection(
        self,
        results,
    ) -> Optional[
        Tuple[str, float, int, int, int, int]
    ]:
        """검출 결과 중 신뢰도가 가장 높은 객체를 선택한다."""

        if not results:
            return None

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return None

        best_detection = None
        best_confidence = -1.0

        for box in result.boxes:
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = str(
                self.model.names[class_id]
            )

            # cup 클래스가 아닌 객체는 무시한다.
            if class_name not in self.TARGET_CLASS_NAMES:
                continue

            x1, y1, x2, y2 = (
                box.xyxy[0]
                .detach()
                .cpu()
                .numpy()
                .astype(int)
                .tolist()
            )

            if confidence > best_confidence:
                best_confidence = confidence
                best_detection = (
                    class_name,
                    confidence,
                    x1,
                    y1,
                    x2,
                    y2,
                )

        return best_detection

    def select_target_pixel(
        self,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> Tuple[int, int]:
        """바운딩 박스에서 Depth 측정용 대표 픽셀을 선택한다."""

        target_u = int((x1 + x2) / 2)

        target_v = int(
            y1
            + self.GRASP_POINT_Y_RATIO
            * (y2 - y1)
        )

        return target_u, target_v

    def update_detection_counter(
        self,
        class_name: str,
    ) -> None:
        """같은 클래스가 연속 검출될 때만 카운터를 증가시킨다."""

        if self.last_target_class == class_name:
            self.consecutive_frames += 1
        else:
            self.last_target_class = class_name
            self.consecutive_frames = 1

    def reset_detection_counter(self) -> None:
        self.consecutive_frames = 0
        self.last_target_class = None

    def get_stable_depth(
        self,
        center_x: int,
        center_y: int,
        depth_frame: np.ndarray,
    ) -> Optional[float]:
        """대표 픽셀 주변의 유효 Depth 중앙값을 반환한다."""

        height, width = depth_frame.shape[:2]
        radius = self.DEPTH_SAMPLE_RADIUS

        x_start = max(0, center_x - radius)
        x_end = min(width, center_x + radius + 1)

        y_start = max(0, center_y - radius)
        y_end = min(height, center_y + radius + 1)

        depth_roi = depth_frame[
            y_start:y_end,
            x_start:x_end,
        ].astype(np.float64)

        valid_depths = depth_roi[
            (depth_roi >= self.MIN_DEPTH_MM)
            & (depth_roi <= self.MAX_DEPTH_MM)
        ]

        if valid_depths.size == 0:
            return None

        return float(np.median(valid_depths))

    def pixel_to_camera_position(
        self,
        pixel_x: int,
        pixel_y: int,
        depth_mm: float,
    ) -> np.ndarray:
        """이미지 픽셀과 Depth를 카메라 기준 XYZ(mm)로 변환한다."""

        camera_x = (
            pixel_x - self.intrinsics["ppx"]
        ) * depth_mm / self.intrinsics["fx"]

        camera_y = (
            pixel_y - self.intrinsics["ppy"]
        ) * depth_mm / self.intrinsics["fy"]

        camera_z = depth_mm

        return np.array(
            [camera_x, camera_y, camera_z],
            dtype=np.float64,
        )

    def get_robot_pose_matrix(
        self,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
    ) -> np.ndarray:
        """로봇 Base→Tool Flange 변환행렬을 생성한다."""

        rotation = Rotation.from_euler(
            "ZYZ",
            [rx, ry, rz],
            degrees=True,
        ).as_matrix()

        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = [x, y, z]

        return transform

    def transform_camera_to_base(
        self,
        camera_position: np.ndarray,
    ) -> np.ndarray:
        """카메라 기준 XYZ를 로봇 베이스 기준 XYZ로 변환한다."""

        current_pose = get_current_tool_flange_posx()

        # API 버전에 따라 pose 자체 또는 (pose, solution) 형태로
        # 반환될 수 있으므로 6축 자세 배열만 추출한다.
        if (
            isinstance(current_pose, (tuple, list))
            and len(current_pose) > 0
            and isinstance(
                current_pose[0],
                (tuple, list, np.ndarray),
            )
        ):
            current_pose = current_pose[0]

        current_pose = np.asarray(
            current_pose,
            dtype=np.float64,
        ).reshape(-1)

        if current_pose.size != 6:
            raise RuntimeError(
                "Tool Flange 자세가 6개 값으로 반환되지 않았습니다: "
                f"{current_pose}"
            )

        self.get_logger().info(
            "현재 Tool Flange 자세: "
            f"x={current_pose[0]:.2f}, "
            f"y={current_pose[1]:.2f}, "
            f"z={current_pose[2]:.2f}, "
            f"rx={current_pose[3]:.2f}, "
            f"ry={current_pose[4]:.2f}, "
            f"rz={current_pose[5]:.2f}"
        )

        base_to_flange = self.get_robot_pose_matrix(
            *current_pose
        )

        base_to_camera = (
            base_to_flange
            @ self.flange_camera
        )

        # 180도 회전 영상 좌표를 실제 카메라 좌표계로 복원한다.
        if self.ROTATE_CAMERA_180:
            camera_position_for_calibration = np.array(
                [
                    -camera_position[0],
                    -camera_position[1],
                    camera_position[2],
                ],
                dtype=np.float64,
            )
        else:
            camera_position_for_calibration = camera_position


        camera_homogeneous = np.append(
            camera_position_for_calibration,
            1.0,
        )

        base_homogeneous = (
            base_to_camera
            @ camera_homogeneous
        )

        return base_homogeneous[:3]

    def publish_position(
        self,
        class_name: str,
        confidence: float,
        base_position: np.ndarray,
    ) -> None:
        msg = BeveragePosition()

        msg.class_name = class_name
        msg.confidence = float(confidence)

        msg.base_x = float(base_position[0])
        msg.base_y = float(base_position[1])
        msg.base_z = float(base_position[2])

        self.position_publisher.publish(msg)

        self.get_logger().info(
            "음료 위치를 퍼블리시했습니다: "
            f"class={msg.class_name}, "
            f"confidence={msg.confidence:.3f}, "
            f"base=({msg.base_x:.2f}, "
            f"{msg.base_y:.2f}, "
            f"{msg.base_z:.2f}) mm"
        )

    def stop_detection(self) -> None:
        self.completed = True
        self.timer.cancel()
        cv2.destroyAllWindows()

        self.get_logger().info(
            "음료 위치 발행을 완료하여 "
            "추가 검출을 중단합니다."
        )

    def destroy_node(self):
        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node(
        "beverage_dsr_node",
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = dsr_node

    detector_node = None

    try:
        global get_current_tool_flange_posx

        from DSR_ROBOT2 import get_current_tool_flange_posx

        detector_node = BeverageDetectorNode()
        rclpy.spin(detector_node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if detector_node is not None:
            detector_node.get_logger().error(
                f"음료 인식 노드 오류: {error}"
            )
        else:
            print(
                f"음료 인식 노드 생성 실패: {error}"
            )

    finally:
        if detector_node is not None:
            detector_node.destroy_node()

        dsr_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
