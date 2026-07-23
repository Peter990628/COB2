#!/usr/bin/env python3
"""상대 깊이 손 추적 토픽으로 Doosan M0609 TCP를 추종하는 ROS 2 노드.

입력 노드
---------
``hand_tracker_node_ver_depth``가 발행하는 다음 토픽을 구독합니다.

* /hand_teleop/hand_position (geometry_msgs/PointStamped)
    - x: 화면 왼쪽 0.0 / 오른쪽 1.0
    - y: 화면 위쪽 0.0 / 아래쪽 1.0
    - z: 중립 깊이 -1.0~+1.0, 카메라 쪽으로 가까워지면 양수
* /hand_teleop/hand_detected (std_msgs/Bool)
* /hand_teleop/depth_valid (std_msgs/Bool)
* /hand_teleop/fist (std_msgs/Bool)

그리퍼 제어
-----------
``/hand_teleop/fist``는 ``hand_tracker_node_ver_depth``에서 이미 여러 프레임
확인된 안정화 상태입니다.

* 손을 펴면 RG2를 80.0 mm, 3.0 N으로 엽니다.
* 주먹을 쥐면 RG2를 2.0 mm, 30.0 N으로 닫습니다.
* 같은 상태가 유지되는 동안에는 같은 Modbus 명령을 반복하지 않습니다.
* 손을 잃으면 물체를 떨어뜨리지 않도록 마지막 그리퍼 상태를 유지합니다.

로봇 좌표 변환
--------------
노드는 실제 동작 모드에서 먼저 ``INITIAL_JOINT_POSE``로 이동한 뒤, 도착한
TCP의 BASE pose를 원점으로 저장합니다. 이후 화면 좌표는 다음처럼 초기 TCP
기준 오프셋으로 변환됩니다.

* 손 화면 +X(오른쪽) -> 로봇 BASE -Y
* 손 화면 +Y(아래쪽) -> 로봇 BASE -Z
* 손 깊이 +Z(카메라 쪽) -> 로봇 BASE -X

X는 중립 위치에서 전진 거리 0 mm이고, 손을 카메라 쪽으로 가까이 가져갈수록
기본 최대 300 mm까지 BASE -X 방향으로 이동합니다. 손이 중립보다 멀어져
입력 Z가 음수가 되면 X 전진 거리는 0 mm로 제한됩니다.

안전
----
* 기본값은 ``dry_run=True``이므로 로봇을 움직이지 않습니다.
* 실제 목표는 매번 초기 TCP pose로부터 계산해 상대 명령의 누적 오차를 막습니다.
* 입력 범위, TCP 오프셋 범위, 한 주기 이동량, 속도와 가속도를 제한합니다.
* 손 유실, 깊이 무효 또는 토픽 타임아웃 시 soft stop을 요청합니다.
* RG2 통신은 별도 스레드에서 실행해 로봇 ``servol`` 주기를 막지 않습니다.
* 이 소프트웨어 제한은 로봇의 안전 설정이나 비상정지 장치를 대신하지 않습니다.

실행 예
-------
먼저 hand_tracker_node_ver_depth를 실행한 뒤 아래 노드를 실행합니다.

Dry run:

    ros2 run hand_teleop hand_follow_robot_node_ver_depth

실제 로봇/RViz:

    ros2 run hand_teleop hand_follow_robot_node_ver_depth --ros-args \
      -p dry_run:=false

그리퍼 값을 실행할 때 바꾸는 예:

    ros2 run hand_teleop hand_follow_robot_node_ver_depth --ros-args \
      -p dry_run:=false \
      -p gripper.open_width_mm:=80.0 \
      -p gripper.open_force_n:=3.0 \
      -p gripper.closed_width_mm:=2.0 \
      -p gripper.closed_force_n:=30.0
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, List, Optional, Sequence, Tuple

import rclpy
from dsr_msgs2.srv import MoveStop
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Bool

import DR_init
from hand_teleop.onrobot import RG2


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


Pose6 = List[float]
HandPosition = Tuple[float, float, float]


def clamp(value: float, minimum: float, maximum: float) -> float:
    """값을 minimum~maximum 범위로 제한합니다."""

    return max(minimum, min(maximum, value))


def map_clamped(
    value: float,
    input_minimum: float,
    input_maximum: float,
    output_at_minimum: float,
    output_at_maximum: float,
) -> float:
    """입력 구간을 출력 구간으로 선형 변환하고 입력 범위를 제한합니다."""

    if input_maximum <= input_minimum:
        raise ValueError("입력 변환 범위의 최대값은 최소값보다 커야 합니다.")

    bounded = clamp(value, input_minimum, input_maximum)
    ratio = (
        (bounded - input_minimum)
        / (input_maximum - input_minimum)
    )
    return (
        output_at_minimum
        + ratio * (output_at_maximum - output_at_minimum)
    )


def limit_linear_step(
    previous_pose: Sequence[float],
    desired_pose: Sequence[float],
    maximum_step_mm: float,
) -> Pose6:
    """X/Y/Z의 한 주기 이동량을 제한하고 원하는 A/B/C는 유지합니다."""

    result = [float(value) for value in desired_pose]
    for axis in range(3):
        delta = float(desired_pose[axis]) - float(previous_pose[axis])
        result[axis] = float(previous_pose[axis]) + clamp(
            delta,
            -maximum_step_mm,
            maximum_step_mm,
        )
    return result


class HandFollowRobotNodeVerDepth(Node):
    """손의 정규화 X/Y/Z를 제한된 M0609 TCP pose로 변환합니다."""

    def __init__(self) -> None:
        # DSR_ROBOT2의 상대 서비스/토픽 이름이 /dsr01 아래로 해석되게 합니다.
        super().__init__(
            "hand_follow_robot_node_ver_depth",
            namespace=ROBOT_ID,
        )

        # ------------------------------------------------------------------
        # 실행 및 초기 자세 설정
        # ------------------------------------------------------------------
        # True이면 로봇과 RG2에 실제 명령을 보내지 않고 계산/제스처 결과만
        # 로그로 확인합니다. 실제 장비 시험 때만 false로 변경합니다.
        self.declare_parameter("dry_run", True)

        # True이면 시작할 때 initial_joint_pose로 movej한 뒤 손 추종을
        # 시작합니다. False이면 현재 TCP 위치를 추종 원점으로 사용합니다.
        self.declare_parameter("move_to_initial_pose", True)

        # 손 추종을 시작하기 전 로봇이 이동할 6축 관절각 [degree]입니다.
        self.declare_parameter(
            "initial_joint_pose",
            [-90.0, 0.0, 90.0, -90.0, 90.0, 90.0],
        )

        # 위 초기 movej에 적용할 관절 속도 [deg/s]와 가속도 [deg/s²]입니다.
        self.declare_parameter("initial_joint_velocity", 20.0)
        self.declare_parameter("initial_joint_acceleration", 40.0)

        # ------------------------------------------------------------------
        # 손 화면 좌표의 유효 구간
        # ------------------------------------------------------------------
        # 화면 가장자리를 사용하지 않으려면 예를 들어 0.1~0.9로 줄일 수
        # 있습니다. 범위 밖 입력은 가장 가까운 경계값으로 제한됩니다.
        self.declare_parameter("screen_x_min", 0.0)
        self.declare_parameter("screen_x_max", 1.0)
        self.declare_parameter("screen_y_min", 0.0)
        self.declare_parameter("screen_y_max", 1.0)

        # ------------------------------------------------------------------
        # 초기 TCP 기준 상대 이동 범위(mm)
        # ------------------------------------------------------------------
        # X는 "전진 거리"의 크기입니다. 기본 x_direction_sign=-1이므로
        # 실제 BASE X 오프셋은 0~-300 mm가 됩니다. 설치 방향상 +X가 벽
        # 방향이라면 실행할 때 x_direction_sign:=1.0으로 바꿉니다.
        self.declare_parameter("x_travel_min_mm", 0.0)
        self.declare_parameter("x_travel_max_mm", 300.0)
        self.declare_parameter("x_direction_sign", -1.0)

        self.declare_parameter("y_offset_min_mm", -300.0)
        self.declare_parameter("y_offset_max_mm", 300.0)
        self.declare_parameter("z_offset_min_mm", -300.0)
        self.declare_parameter("z_offset_max_mm", 300.0)

        # ------------------------------------------------------------------
        # servol 제어와 입력 watchdog
        # ------------------------------------------------------------------
        # 10 Hz이면 0.1초마다 최신 손 좌표를 로봇 목표로 계산합니다.
        self.declare_parameter("control_rate_hz", 10.0)

        # 이 시간 동안 새 위치 토픽이 없으면 손 추종을 감속 정지합니다.
        self.declare_parameter("hand_timeout_sec", 0.30)

        # 한 제어 주기에 명령 위치가 축별로 변할 수 있는 최대값 [mm]입니다.
        self.declare_parameter("maximum_step_mm", 5.0)

        # servol의 최대 TCP 선속도 [mm/s]와 회전속도 [deg/s]입니다.
        self.declare_parameter("servo_linear_velocity", 30.0)
        self.declare_parameter("servo_rotational_velocity", 10.0)

        # servol의 최대 TCP 선가속도 [mm/s²]와 회전가속도 [deg/s²]입니다.
        self.declare_parameter("servo_linear_acceleration", 60.0)
        self.declare_parameter("servo_rotational_acceleration", 30.0)

        # servol이 각 최신 목표에 도달하도록 요청하는 시간 [second]입니다.
        self.declare_parameter("servo_reach_time_sec", 0.15)

        # 추종 상태 로그가 너무 많이 출력되지 않도록 하는 간격입니다.
        self.declare_parameter("status_log_period_sec", 1.0)

        # ------------------------------------------------------------------
        # OnRobot RG2 설정
        # ------------------------------------------------------------------
        # False로 실행하면 주먹 토픽은 받지만 RG2 명령은 전혀 보내지 않습니다.
        self.declare_parameter("gripper.enabled", True)

        # Compute Box 또는 Tool Changer Modbus TCP 주소와 포트입니다.
        self.declare_parameter("gripper.ip", "192.168.1.1")
        self.declare_parameter("gripper.port", 502)

        # RG2 명령은 사람이 읽기 쉬운 mm와 N 단위로 설정합니다.
        # 내부 Modbus 전송 직전에 각각 0.1 mm, 0.1 N raw 단위로 변환됩니다.
        #
        # 주의: 80.0 mm는 raw width=800입니다. RG2의 전체 폭 범위는
        # 0~110 mm이고 공식 파지력 범위는 3~40 N입니다.
        self.declare_parameter("gripper.open_width_mm", 80.0)
        self.declare_parameter("gripper.open_force_n", 30.0)
        self.declare_parameter("gripper.closed_width_mm", 2.0)
        self.declare_parameter("gripper.closed_force_n", 30.0)

        # Modbus 한 번의 통신 제한시간과 그리퍼 동작 완료 대기 제한시간입니다.
        self.declare_parameter("gripper.modbus_timeout_sec", 1.0)
        self.declare_parameter("gripper.motion_timeout_sec", 5.0)

        # busy 상태를 다시 확인하는 주기입니다. 이 대기는 별도 스레드에서
        # 실행되므로 로봇의 servol 타이머를 멈추지 않습니다.
        self.declare_parameter("gripper.poll_period_sec", 0.05)

        # 손이 재검출된 직후에는 fist 토픽의 안정화 결과를 기다립니다.
        # 0.35초는 기본 30 FPS, fist_confirm_frames=8보다 조금 긴 값입니다.
        self.declare_parameter("gripper.hand_reacquire_delay_sec", 0.35)

        # ------------------------------------------------------------------
        # 구독 토픽
        # ------------------------------------------------------------------
        self.declare_parameter(
            "position_topic",
            "/hand_teleop/hand_position",
        )
        self.declare_parameter(
            "detected_topic",
            "/hand_teleop/hand_detected",
        )
        self.declare_parameter(
            "depth_valid_topic",
            "/hand_teleop/depth_valid",
        )
        self.declare_parameter("fist_topic", "/hand_teleop/fist")

        self._read_and_validate_parameters()

        # 가장 최근에 받은 손 상태입니다. 구독 콜백에서는 로봇을 움직이지
        # 않고 값만 저장하며, 실제 제어는 고정 주기의 타이머에서 수행합니다.
        self._latest_hand_position: Optional[HandPosition] = None
        self._last_position_received_at: Optional[float] = None
        self._hand_detected = False
        self._depth_valid = False
        self._fist = False
        self._hand_session_started_at: Optional[float] = None
        self._last_fist_received_at: Optional[float] = None

        self._origin_pose: Optional[Pose6] = None
        self._last_commanded_pose: Optional[Pose6] = None
        self._motion_active = False
        self._faulted = False
        self._last_block_reason: Optional[str] = None
        self._last_status_log_at = 0.0
        self._last_stop_warning_at = 0.0

        # 실제 모드에서 채워지는 DSR 함수 참조입니다.
        self._servol: Optional[Callable] = None
        self._posx: Optional[Callable] = None

        # RG2 Modbus 객체는 실제 모드에서만 생성합니다. 주먹 콜백에서 직접
        # 통신하면 servol 타이머가 지연될 수 있으므로 Condition과 작업 스레드를
        # 사용해 원하는 상태만 전달합니다.
        self._gripper: Optional[RG2] = None
        self._gripper_condition = threading.Condition()
        self._gripper_requested_closed: Optional[bool] = None
        self._gripper_commanded_closed: Optional[bool] = None
        self._gripper_shutdown_requested = False
        self._gripper_cleaned_up = False
        self._gripper_worker: Optional[threading.Thread] = None

        position_topic = str(
            self.get_parameter("position_topic").value
        )
        detected_topic = str(
            self.get_parameter("detected_topic").value
        )
        depth_valid_topic = str(
            self.get_parameter("depth_valid_topic").value
        )
        fist_topic = str(self.get_parameter("fist_topic").value)

        self.create_subscription(
            PointStamped,
            position_topic,
            self._position_callback,
            10,
        )
        self.create_subscription(
            Bool,
            detected_topic,
            self._detected_callback,
            10,
        )
        self.create_subscription(
            Bool,
            depth_valid_topic,
            self._depth_valid_callback,
            10,
        )
        self.create_subscription(
            Bool,
            fist_topic,
            self._fist_callback,
            10,
        )

        self._stop_client = self.create_client(
            MoveStop,
            "motion/move_stop",
        )

        try:
            if self._dry_run:
                self.get_logger().warning(
                    "DRY RUN: 로봇 초기 이동, servol 및 RG2 명령을 "
                    "실제로 보내지 않습니다."
                )
            else:
                # RG2 연결을 먼저 확인한 뒤 로봇을 초기 위치로 움직입니다.
                # 그리퍼 연결 실패 상태에서 로봇만 움직이는 일을 막기 위함입니다.
                self._initialise_gripper()
                self._initialise_robot()

            # Dry run에서도 제스처에 따른 OPEN/CLOSE 계산 로그를 확인할 수
            # 있도록 작업 스레드는 시작합니다.
            self._start_gripper_worker()
        except Exception:
            # 생성 도중 실패하면 열린 Modbus 소켓을 남기지 않습니다.
            self._shutdown_gripper()
            raise

        self._control_timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_tick,
        )

        self.get_logger().info(
            "손 추종 제어 준비 완료: "
            f"initial_joint={self._initial_joint_pose}, "
            f"X travel={self._x_travel_min_mm:.1f}~"
            f"{self._x_travel_max_mm:.1f} mm "
            f"(sign={self._x_direction_sign:+.0f}), "
            f"Y={self._y_offset_min_mm:.1f}~"
            f"{self._y_offset_max_mm:.1f} mm, "
            f"Z={self._z_offset_min_mm:.1f}~"
            f"{self._z_offset_max_mm:.1f} mm"
        )
        self.get_logger().warning(
            "현재 ±300 mm Y/Z 범위는 큰 시험 범위입니다. "
            "실제 장비에서는 주변 충돌과 네 모서리 도달 가능성을 먼저 "
            "확인하고 더 작은 값부터 시험하세요."
        )
        if self._gripper_enabled:
            self.get_logger().info(
                "RG2 손 제스처 제어: "
                f"OPEN={self._gripper_open_width_mm:.1f} mm/"
                f"{self._gripper_open_force_n:.1f} N, "
                f"FIST={self._gripper_closed_width_mm:.1f} mm/"
                f"{self._gripper_closed_force_n:.1f} N. "
                "손 유실 시에는 마지막 그리퍼 상태를 유지합니다."
            )
        else:
            self.get_logger().warning(
                "gripper.enabled=False: 주먹 토픽을 받아도 RG2를 "
                "동작시키지 않습니다."
            )

    def _read_and_validate_parameters(self) -> None:
        """ROS 파라미터를 멤버 변수로 읽고 잘못된 설정을 차단합니다."""

        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._move_to_initial_pose = bool(
            self.get_parameter("move_to_initial_pose").value
        )
        self._initial_joint_pose = [
            float(value)
            for value in self.get_parameter("initial_joint_pose").value
        ]
        self._initial_joint_velocity = float(
            self.get_parameter("initial_joint_velocity").value
        )
        self._initial_joint_acceleration = float(
            self.get_parameter("initial_joint_acceleration").value
        )

        self._screen_x_min = float(
            self.get_parameter("screen_x_min").value
        )
        self._screen_x_max = float(
            self.get_parameter("screen_x_max").value
        )
        self._screen_y_min = float(
            self.get_parameter("screen_y_min").value
        )
        self._screen_y_max = float(
            self.get_parameter("screen_y_max").value
        )

        self._x_travel_min_mm = float(
            self.get_parameter("x_travel_min_mm").value
        )
        self._x_travel_max_mm = float(
            self.get_parameter("x_travel_max_mm").value
        )
        self._x_direction_sign = float(
            self.get_parameter("x_direction_sign").value
        )
        self._y_offset_min_mm = float(
            self.get_parameter("y_offset_min_mm").value
        )
        self._y_offset_max_mm = float(
            self.get_parameter("y_offset_max_mm").value
        )
        self._z_offset_min_mm = float(
            self.get_parameter("z_offset_min_mm").value
        )
        self._z_offset_max_mm = float(
            self.get_parameter("z_offset_max_mm").value
        )

        self._control_rate_hz = float(
            self.get_parameter("control_rate_hz").value
        )
        self._hand_timeout_sec = float(
            self.get_parameter("hand_timeout_sec").value
        )
        self._maximum_step_mm = float(
            self.get_parameter("maximum_step_mm").value
        )
        self._servo_linear_velocity = float(
            self.get_parameter("servo_linear_velocity").value
        )
        self._servo_rotational_velocity = float(
            self.get_parameter("servo_rotational_velocity").value
        )
        self._servo_linear_acceleration = float(
            self.get_parameter("servo_linear_acceleration").value
        )
        self._servo_rotational_acceleration = float(
            self.get_parameter("servo_rotational_acceleration").value
        )
        self._servo_reach_time_sec = float(
            self.get_parameter("servo_reach_time_sec").value
        )
        self._status_log_period_sec = float(
            self.get_parameter("status_log_period_sec").value
        )

        # RG2 사용 여부와 Modbus 연결 정보를 읽습니다.
        self._gripper_enabled = bool(
            self.get_parameter("gripper.enabled").value
        )
        self._gripper_ip = str(
            self.get_parameter("gripper.ip").value
        ).strip()
        self._gripper_port = int(
            self.get_parameter("gripper.port").value
        )

        # 아래 네 값은 실제 단위가 mm와 N이므로 raw 값보다 설정 의도가
        # 명확합니다. RG2 드라이버가 전송 직전에 10을 곱해 변환합니다.
        self._gripper_open_width_mm = float(
            self.get_parameter("gripper.open_width_mm").value
        )
        self._gripper_open_force_n = float(
            self.get_parameter("gripper.open_force_n").value
        )
        self._gripper_closed_width_mm = float(
            self.get_parameter("gripper.closed_width_mm").value
        )
        self._gripper_closed_force_n = float(
            self.get_parameter("gripper.closed_force_n").value
        )

        self._gripper_modbus_timeout_sec = float(
            self.get_parameter("gripper.modbus_timeout_sec").value
        )
        self._gripper_motion_timeout_sec = float(
            self.get_parameter("gripper.motion_timeout_sec").value
        )
        self._gripper_poll_period_sec = float(
            self.get_parameter("gripper.poll_period_sec").value
        )
        self._gripper_hand_reacquire_delay_sec = float(
            self.get_parameter(
                "gripper.hand_reacquire_delay_sec"
            ).value
        )

        numeric_values = (
            self._initial_joint_pose
            + [
                self._initial_joint_velocity,
                self._initial_joint_acceleration,
                self._screen_x_min,
                self._screen_x_max,
                self._screen_y_min,
                self._screen_y_max,
                self._x_travel_min_mm,
                self._x_travel_max_mm,
                self._x_direction_sign,
                self._y_offset_min_mm,
                self._y_offset_max_mm,
                self._z_offset_min_mm,
                self._z_offset_max_mm,
                self._control_rate_hz,
                self._hand_timeout_sec,
                self._maximum_step_mm,
                self._servo_linear_velocity,
                self._servo_rotational_velocity,
                self._servo_linear_acceleration,
                self._servo_rotational_acceleration,
                self._servo_reach_time_sec,
                self._status_log_period_sec,
                float(self._gripper_port),
                self._gripper_open_width_mm,
                self._gripper_open_force_n,
                self._gripper_closed_width_mm,
                self._gripper_closed_force_n,
                self._gripper_modbus_timeout_sec,
                self._gripper_motion_timeout_sec,
                self._gripper_poll_period_sec,
                self._gripper_hand_reacquire_delay_sec,
            ]
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("모든 로봇 제어 파라미터는 유한한 수여야 합니다.")
        if len(self._initial_joint_pose) != 6:
            raise ValueError("initial_joint_pose는 관절각 6개여야 합니다.")
        if (
            self._initial_joint_velocity <= 0.0
            or self._initial_joint_acceleration <= 0.0
        ):
            raise ValueError("초기 movej 속도와 가속도는 0보다 커야 합니다.")
        if not 0.0 <= self._screen_x_min < self._screen_x_max <= 1.0:
            raise ValueError("screen_x_min/max는 0~1의 올바른 범위여야 합니다.")
        if not 0.0 <= self._screen_y_min < self._screen_y_max <= 1.0:
            raise ValueError("screen_y_min/max는 0~1의 올바른 범위여야 합니다.")
        if not 0.0 <= self._x_travel_min_mm < self._x_travel_max_mm:
            raise ValueError(
                "X 이동 거리는 0 <= minimum < maximum이어야 합니다."
            )
        if self._x_direction_sign not in (-1.0, 1.0):
            raise ValueError("x_direction_sign은 -1.0 또는 1.0이어야 합니다.")
        if self._y_offset_min_mm >= self._y_offset_max_mm:
            raise ValueError("Y offset 최소값은 최대값보다 작아야 합니다.")
        if self._z_offset_min_mm >= self._z_offset_max_mm:
            raise ValueError("Z offset 최소값은 최대값보다 작아야 합니다.")
        if (
            self._control_rate_hz <= 0.0
            or self._hand_timeout_sec <= 0.0
            or self._maximum_step_mm <= 0.0
            or self._servo_linear_velocity <= 0.0
            or self._servo_rotational_velocity <= 0.0
            or self._servo_linear_acceleration <= 0.0
            or self._servo_rotational_acceleration <= 0.0
            or self._servo_reach_time_sec <= 0.0
            or self._status_log_period_sec <= 0.0
        ):
            raise ValueError("제어 주기·제한·속도·가속도는 0보다 커야 합니다.")

        # RG2 공식 사양:
        #   폭 0~110 mm, 힘 3~40 N
        # 사용자가 실수로 raw 값(예: 800)을 mm 자리에 넣는 것도 여기서
        # 즉시 차단해 잘못된 Modbus 명령을 보내지 않게 합니다.
        gripper_values = (
            ("gripper.open_width_mm", self._gripper_open_width_mm),
            ("gripper.closed_width_mm", self._gripper_closed_width_mm),
        )
        for name, value in gripper_values:
            if not 0.0 <= value <= 110.0:
                raise ValueError(f"{name}는 RG2 범위 0~110 mm여야 합니다.")

        gripper_forces = (
            ("gripper.open_force_n", self._gripper_open_force_n),
            ("gripper.closed_force_n", self._gripper_closed_force_n),
        )
        for name, value in gripper_forces:
            if not 3.0 <= value <= 40.0:
                raise ValueError(f"{name}는 RG2 범위 3~40 N이어야 합니다.")

        if (
            self._gripper_closed_width_mm
            >= self._gripper_open_width_mm
        ):
            raise ValueError(
                "gripper.closed_width_mm는 open_width_mm보다 작아야 합니다."
            )
        if not self._gripper_ip:
            raise ValueError("gripper.ip는 빈 문자열일 수 없습니다.")
        if not 1 <= self._gripper_port <= 65535:
            raise ValueError("gripper.port는 1~65535 범위여야 합니다.")
        if (
            self._gripper_modbus_timeout_sec <= 0.0
            or self._gripper_motion_timeout_sec <= 0.0
            or self._gripper_poll_period_sec <= 0.0
            or self._gripper_hand_reacquire_delay_sec < 0.0
        ):
            raise ValueError(
                "RG2 timeout/poll 값은 0보다 커야 하고 "
                "hand_reacquire_delay_sec는 0 이상이어야 합니다."
            )

    def _initialise_gripper(self) -> None:
        """실제 동작 모드에서 OnRobot RG2 Modbus TCP 연결을 엽니다."""

        if not self._gripper_enabled:
            return

        self.get_logger().info(
            "RG2 Modbus TCP 연결을 시작합니다: "
            f"{self._gripper_ip}:{self._gripper_port}"
        )
        self._gripper = RG2(
            ip=self._gripper_ip,
            port=self._gripper_port,
            timeout_sec=self._gripper_modbus_timeout_sec,
        )
        self.get_logger().info("RG2 Modbus TCP 연결을 확인했습니다.")

    def _start_gripper_worker(self) -> None:
        """그리퍼 통신 전용 작업 스레드를 한 번 시작합니다."""

        if not self._gripper_enabled or self._gripper_worker is not None:
            return

        self._gripper_worker = threading.Thread(
            target=self._gripper_worker_main,
            name="rg2_gesture_worker",
            daemon=True,
        )
        self._gripper_worker.start()

    def _request_gripper_state(self, closed: bool) -> None:
        """작업 스레드에 최신 OPEN/CLOSE 목표를 비동기로 전달합니다."""

        if not self._gripper_enabled:
            return

        with self._gripper_condition:
            # 같은 목표가 계속 들어오면 Condition을 깨우거나 Modbus 명령을
            # 반복하지 않습니다. 오직 OPEN↔FIST 전환만 처리합니다.
            if self._gripper_requested_closed == closed:
                return
            self._gripper_requested_closed = closed
            self._gripper_condition.notify_all()

    def _update_gripper_request_from_hand(self, now: float) -> None:
        """유효한 새 손 세션의 안정화 fist 상태만 RG2 목표로 사용합니다."""

        if not self._gripper_enabled or not self._hand_detected:
            # 손을 잃었을 때 OPEN을 보내면 들고 있던 물체가 떨어질 수
            # 있으므로 아무 명령도 보내지 않고 마지막 상태를 유지합니다.
            return
        if (
            self._hand_session_started_at is None
            or self._last_fist_received_at is None
            or self._last_fist_received_at
            < self._hand_session_started_at
        ):
            return
        if (
            now - self._hand_session_started_at
            < self._gripper_hand_reacquire_delay_sec
        ):
            # 손을 처음 잡은 직후 stable_fist=False가 잠시 발행되는 동안
            # 실제 주먹인데도 RG2가 먼저 열리는 것을 방지합니다.
            return

        self._request_gripper_state(closed=self._fist)

    def _wait_until_gripper_idle(self) -> bool:
        """RG2 busy가 해제될 때까지 작업 스레드에서 제한 시간만 기다립니다."""

        if self._gripper is None:
            raise RuntimeError("RG2 연결 객체가 없습니다.")

        deadline = time.monotonic() + self._gripper_motion_timeout_sec
        while time.monotonic() < deadline:
            with self._gripper_condition:
                if self._gripper_shutdown_requested:
                    return False

            status = self._gripper.get_status()
            if status.safety_fault:
                raise RuntimeError(
                    "RG2 안전 스위치 또는 안전 회로 오류가 감지됐습니다."
                )
            if not status.busy:
                return True

            time.sleep(self._gripper_poll_period_sec)

        raise TimeoutError(
            "RG2의 이전 동작이 제한 시간 안에 끝나지 않았습니다."
        )

    def _gripper_worker_main(self) -> None:
        """OPEN/CLOSE 상태 전환을 순서대로 처리하는 Modbus 작업 루프입니다."""

        while True:
            with self._gripper_condition:
                self._gripper_condition.wait_for(
                    lambda: (
                        self._gripper_shutdown_requested
                        or (
                            self._gripper_requested_closed is not None
                            and self._gripper_requested_closed
                            != self._gripper_commanded_closed
                        )
                    )
                )
                if self._gripper_shutdown_requested:
                    return
                requested_closed = self._gripper_requested_closed

            assert requested_closed is not None

            # Dry run은 장치에 연결하지 않고 상태 전환과 설정값만 보여줍니다.
            if self._dry_run:
                description = "CLOSE(FIST)" if requested_closed else "OPEN"
                width_mm, force_n = self._gripper_target(requested_closed)
                self.get_logger().info(
                    "DRY RUN RG2: "
                    f"{description}, width={width_mm:.1f} mm, "
                    f"force={force_n:.1f} N"
                )
                with self._gripper_condition:
                    self._gripper_commanded_closed = requested_closed
                continue

            try:
                if not self._wait_until_gripper_idle():
                    return

                # 이전 동작을 기다리는 사이 손 모양이 다시 바뀌었을 수
                # 있으므로 대기 후 가장 최신 목표를 다시 읽습니다.
                with self._gripper_condition:
                    if self._gripper_shutdown_requested:
                        return
                    requested_closed = self._gripper_requested_closed
                    if (
                        requested_closed is None
                        or requested_closed
                        == self._gripper_commanded_closed
                    ):
                        continue

                width_mm, force_n = self._gripper_target(
                    requested_closed
                )
                assert self._gripper is not None
                self._gripper.move_gripper_mm_n(
                    width_mm=width_mm,
                    force_n=force_n,
                )

                description = "CLOSE(FIST)" if requested_closed else "OPEN"
                self.get_logger().info(
                    "RG2 명령 전송: "
                    f"{description}, width={width_mm:.1f} mm, "
                    f"force={force_n:.1f} N"
                )
                with self._gripper_condition:
                    self._gripper_commanded_closed = requested_closed
            except Exception as error:
                # 그리퍼 통신 장애가 생겼는데 로봇만 계속 움직이지 않도록
                # 공통 fault를 올립니다. 다음 control tick에서 soft stop합니다.
                self.get_logger().error(f"RG2 제어 오류: {error}")
                self._faulted = True
                return

    def _gripper_target(self, closed: bool) -> Tuple[float, float]:
        """손 상태에 해당하는 RG2 목표 폭 [mm]과 힘 [N]을 반환합니다."""

        if closed:
            return (
                self._gripper_closed_width_mm,
                self._gripper_closed_force_n,
            )
        return (
            self._gripper_open_width_mm,
            self._gripper_open_force_n,
        )

    def _shutdown_gripper(self) -> None:
        """작업 스레드를 끝낸 뒤 RG2 Modbus 소켓을 안전하게 닫습니다."""

        if self._gripper_cleaned_up:
            return
        self._gripper_cleaned_up = True

        with self._gripper_condition:
            self._gripper_shutdown_requested = True
            self._gripper_condition.notify_all()

        worker = self._gripper_worker
        if worker is not None and worker.is_alive():
            worker.join(
                timeout=self._gripper_modbus_timeout_sec + 1.0
            )
            if worker.is_alive():
                self.get_logger().warning(
                    "RG2 작업 스레드가 제한 시간 안에 종료되지 않았습니다."
                )

        if self._gripper is not None:
            self._gripper.close_connection()
            self._gripper = None

    def _initialise_robot(self) -> None:
        """DSR 모듈을 초기화하고 초기 관절/TCP 기준 자세를 설정합니다."""

        # DSR_ROBOT2는 import 순간 DR_init.__dsr__node를 사용하여 서비스와
        # publisher를 생성하므로 반드시 import 전에 현재 노드를 넣어야 합니다.
        #
        # 주의: 클래스 메서드 안에서 ``DR_init.__dsr__node = self``라고 직접
        # 쓰면 Python의 이중 밑줄 name mangling 때문에 실제 속성 이름이
        # ``_HandFollowRobotNodeVerDepth__dsr__node``로 바뀝니다. setattr()로
        # 정확히 "__dsr__node" 속성을 설정해야 DSR_ROBOT2가 같은 노드를 봅니다.
        setattr(DR_init, "__dsr__node", self)
        if getattr(DR_init, "__dsr__node", None) is not self:
            raise RuntimeError("DR_init.__dsr__node 설정에 실패했습니다.")

        from DSR_ROBOT2 import (
            DR_BASE,
            get_current_posx,
            movej,
            posj,
            posx,
            servol,
        )

        if self._move_to_initial_pose:
            self.get_logger().info(
                "초기 관절 자세로 이동합니다: "
                f"{self._initial_joint_pose}"
            )
            result = movej(
                posj(self._initial_joint_pose),
                vel=self._initial_joint_velocity,
                acc=self._initial_joint_acceleration,
            )
            if result != 0:
                raise RuntimeError(
                    f"초기 관절 자세 movej 실패, 반환값={result}"
                )
        else:
            self.get_logger().warning(
                "move_to_initial_pose=False: 현재 TCP를 추종 원점으로 사용합니다."
            )

        current_pose_result = get_current_posx(ref=DR_BASE)
        if (
            not isinstance(current_pose_result, tuple)
            or len(current_pose_result) < 1
            or current_pose_result[0] is None
        ):
            raise RuntimeError(
                f"초기 TCP pose를 읽지 못했습니다: {current_pose_result}"
            )

        origin_pose = [float(value) for value in current_pose_result[0]]
        if len(origin_pose) != 6 or not all(
            math.isfinite(value) for value in origin_pose
        ):
            raise RuntimeError(
                f"초기 TCP pose가 올바르지 않습니다: {origin_pose}"
            )

        self._origin_pose = origin_pose
        self._last_commanded_pose = origin_pose.copy()
        self._servol = servol
        self._posx = posx
        self.get_logger().info(
            "초기 TCP pose(BASE)를 추종 원점으로 저장했습니다: "
            f"{[round(value, 3) for value in origin_pose]}"
        )

    def _position_callback(self, message: PointStamped) -> None:
        """가장 최근의 유효한 정규화 손 좌표와 수신 시간을 저장합니다."""

        values = (
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                f"유한하지 않은 손 좌표를 무시합니다: {values}"
            )
            return
        if (
            not 0.0 <= values[0] <= 1.0
            or not 0.0 <= values[1] <= 1.0
            or not -1.0 <= values[2] <= 1.0
        ):
            self.get_logger().warning(
                f"허용 범위 밖의 손 좌표를 무시합니다: {values}"
            )
            return

        self._latest_hand_position = values
        self._last_position_received_at = time.monotonic()

    def _detected_callback(self, message: Bool) -> None:
        """손 검출 여부와 새 손 세션의 시작 시간을 저장합니다."""

        detected = bool(message.data)
        if detected and not self._hand_detected:
            # 새 손이 검출된 시각을 기록해 이전 손 세션의 fist=False가
            # 새 세션의 OPEN 명령으로 잘못 사용되지 않게 합니다.
            self._hand_session_started_at = time.monotonic()
        elif not detected:
            self._hand_session_started_at = None

        self._hand_detected = detected

    def _depth_valid_callback(self, message: Bool) -> None:
        """상대 깊이 보정 및 현재 깊이 유효 여부를 저장합니다."""

        self._depth_valid = bool(message.data)

    def _fist_callback(self, message: Bool) -> None:
        """그리퍼 동작에 사용할 안정화된 주먹 상태와 시간을 저장합니다."""

        self._fist = bool(message.data)
        self._last_fist_received_at = time.monotonic()

    def _tracking_block_reason(self, now: float) -> Optional[str]:
        """현재 로봇 추종을 중단해야 하는 이유를 반환합니다."""

        if self._faulted:
            return "로봇 제어 fault가 발생해 재시작이 필요합니다"
        if not self._hand_detected:
            return "손이 검출되지 않았습니다"
        if not self._depth_valid:
            return "상대 깊이가 아직 유효하지 않습니다"
        if (
            self._latest_hand_position is None
            or self._last_position_received_at is None
        ):
            return "손 위치 토픽을 아직 받지 못했습니다"

        age = now - self._last_position_received_at
        if age > self._hand_timeout_sec:
            return (
                f"손 위치 토픽 timeout "
                f"({age:.3f}s > {self._hand_timeout_sec:.3f}s)"
            )
        return None

    def _hand_to_offsets(
        self,
        hand_position: HandPosition,
    ) -> Tuple[float, float, float]:
        """손 정규화 X/Y/Z를 초기 TCP 기준 BASE 오프셋으로 변환합니다."""

        hand_x, hand_y, hand_depth = hand_position

        # 중립보다 가까운 깊이만 0~1 전진 명령으로 사용합니다. 따라서 손을
        # 뒤로 빼면 초기 X로 복귀하지만 초기 자세 반대 방향으로 넘어가지는
        # 않습니다.
        forward_depth = clamp(hand_depth, 0.0, 1.0)
        x_travel = map_clamped(
            forward_depth,
            0.0,
            1.0,
            self._x_travel_min_mm,
            self._x_travel_max_mm,
        )
        x_offset = self._x_direction_sign * x_travel

        # 화면 왼쪽 -> +Y, 화면 오른쪽 -> -Y
        y_offset = map_clamped(
            hand_x,
            self._screen_x_min,
            self._screen_x_max,
            self._y_offset_max_mm,
            self._y_offset_min_mm,
        )

        # 화면 위쪽 -> +Z, 화면 아래쪽 -> -Z
        z_offset = map_clamped(
            hand_y,
            self._screen_y_min,
            self._screen_y_max,
            self._z_offset_max_mm,
            self._z_offset_min_mm,
        )
        return x_offset, y_offset, z_offset

    def _make_desired_pose(
        self,
        offsets: Tuple[float, float, float],
    ) -> Pose6:
        """초기 TCP의 A/B/C를 유지한 절대 BASE 목표 pose를 만듭니다."""

        if self._origin_pose is None:
            raise RuntimeError("초기 TCP 원점 pose가 설정되지 않았습니다.")

        x_offset, y_offset, z_offset = offsets
        return [
            self._origin_pose[0] + x_offset,
            self._origin_pose[1] + y_offset,
            self._origin_pose[2] + z_offset,
            self._origin_pose[3],
            self._origin_pose[4],
            self._origin_pose[5],
        ]

    def _request_soft_stop(self, reason: str) -> None:
        """이동 중이었다면 비동기 soft stop을 한 번 요청합니다."""

        if self._dry_run or not self._motion_active:
            return

        self._motion_active = False
        if not self._stop_client.service_is_ready():
            now = time.monotonic()
            if now - self._last_stop_warning_at >= 2.0:
                self.get_logger().warning(
                    "motion/move_stop 서비스를 사용할 수 없어 "
                    f"soft stop을 요청하지 못했습니다: {reason}"
                )
                self._last_stop_warning_at = now
            return

        request = MoveStop.Request()
        request.stop_mode = 2  # DR_SSTOP: 감속 정지
        future = self._stop_client.call_async(request)

        def _stop_done(completed_future) -> None:
            try:
                response = completed_future.result()
                if response is None or not response.success:
                    self.get_logger().error(
                        f"soft stop 응답 실패: {reason}"
                    )
                else:
                    self.get_logger().info(
                        f"손 추종 soft stop 완료: {reason}"
                    )
            except Exception as error:
                self.get_logger().error(
                    f"soft stop 서비스 오류: {error}"
                )

        future.add_done_callback(_stop_done)

    def _log_status(
        self,
        now: float,
        offsets: Tuple[float, float, float],
        target_pose: Optional[Sequence[float]],
    ) -> None:
        """제어 상태를 너무 자주 출력하지 않도록 주기적으로 기록합니다."""

        if now - self._last_status_log_at < self._status_log_period_sec:
            return
        self._last_status_log_at = now

        hand = self._latest_hand_position
        if hand is None:
            return

        target_text = (
            "DRY_RUN"
            if target_pose is None
            else str([round(value, 2) for value in target_pose])
        )
        self.get_logger().info(
            "추종: "
            f"hand=({hand[0]:.3f}, {hand[1]:.3f}, {hand[2]:+.3f}), "
            f"offset=({offsets[0]:+.1f}, "
            f"{offsets[1]:+.1f}, {offsets[2]:+.1f}) mm, "
            f"fist={self._fist}, target={target_text}"
        )

    def _control_tick(self) -> None:
        """고정 주기로 안전 상태를 확인하고 최신 TCP 목표를 전송합니다."""

        now = time.monotonic()

        # RG2 요청은 로봇 추종 가능 여부와 별개로 손 검출과 fist 상태만
        # 사용합니다. 예를 들어 깊이 보정 중이어도 손이 정상 검출되면
        # OPEN/CLOSE 동작은 시험할 수 있습니다.
        self._update_gripper_request_from_hand(now)

        block_reason = self._tracking_block_reason(now)
        if block_reason is not None:
            if block_reason != self._last_block_reason:
                self.get_logger().info(
                    f"손 추종 대기/중지: {block_reason}"
                )
                self._last_block_reason = block_reason
            self._request_soft_stop(block_reason)
            return

        if self._last_block_reason is not None:
            self.get_logger().info("유효한 손 입력을 받아 추종을 시작합니다.")
            self._last_block_reason = None

        hand_position = self._latest_hand_position
        assert hand_position is not None
        offsets = self._hand_to_offsets(hand_position)

        if self._dry_run:
            self._log_status(now, offsets, target_pose=None)
            return

        if (
            self._servol is None
            or self._posx is None
            or self._origin_pose is None
            or self._last_commanded_pose is None
        ):
            self._faulted = True
            self.get_logger().error(
                "DSR 로봇 제어 함수 또는 초기 TCP pose가 준비되지 않았습니다."
            )
            return

        desired_pose = self._make_desired_pose(offsets)
        limited_pose = limit_linear_step(
            self._last_commanded_pose,
            desired_pose,
            self._maximum_step_mm,
        )

        try:
            result = self._servol(
                self._posx(limited_pose),
                vel=[
                    self._servo_linear_velocity,
                    self._servo_rotational_velocity,
                ],
                acc=[
                    self._servo_linear_acceleration,
                    self._servo_rotational_acceleration,
                ],
                time=self._servo_reach_time_sec,
            )
            if result != 0:
                raise RuntimeError(f"servol 반환값={result}")

            self._last_commanded_pose = limited_pose
            self._motion_active = True
            self._log_status(now, offsets, target_pose=limited_pose)
        except Exception as error:
            self._faulted = True
            self.get_logger().error(
                f"servol 명령 중 오류가 발생했습니다: {error}"
            )
            self._request_soft_stop("servol 오류")

    def request_shutdown_stop(self) -> None:
        """노드 종료 전에 가능한 경우 마지막 soft stop을 요청합니다."""

        timer = getattr(self, "_control_timer", None)
        if timer is not None:
            timer.cancel()
        self._request_soft_stop("노드 종료")

    def destroy_node(self) -> bool:
        """타이머·RG2 연결을 정리한 뒤 ROS 노드를 종료합니다."""

        timer = getattr(self, "_control_timer", None)
        if timer is not None:
            timer.cancel()
        self._shutdown_gripper()
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    """ROS 2 console_scripts 진입점입니다."""

    rclpy.init(args=args)
    node: Optional[HandFollowRobotNodeVerDepth] = None

    try:
        node = HandFollowRobotNodeVerDepth()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.request_shutdown_stop()
                # 비동기 stop 요청이 executor에서 처리될 짧은 기회를 줍니다.
                rclpy.spin_once(node, timeout_sec=0.2)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
