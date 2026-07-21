#!/usr/bin/env python3
"""Pick a selected beverage and place it at a configured position.

The prototype command topic uses ``std_msgs/msg/String``. Both formats below
are accepted::

    {"beverage": "cola", "pose": [x, y, z, a, b, c]}
    콜라,x,y,z,a,b,c

The pose must already be a Doosan BASE-frame TCP pose in mm/degree. Camera
coordinates in metres or a RealSense optical frame must be transformed before
publishing this command.
"""
# beverage_test
# =============================================================================
# 실행할 때 바꿀 수 있는 설정 가이드
# =============================================================================
#
# 이 파일의 기본값을 직접 수정할 수도 있지만, 테스트할 때는 ROS parameter로
# 덮어쓰는 방법을 권장합니다. 그러면 코드를 다시 수정하지 않고 거리, 속도,
# 배치 위치와 그리퍼 설정을 바꿀 수 있습니다.
#
# 1. 먼저 실제 장비가 움직이지 않는 dry-run으로 경로를 확인합니다.
#
#    ros2 run cobot_pjt beverage_test --ros-args \
#      -p dry_run:=true \
#      -p pick_approach_mm:=100.0 \
#      -p pick_descent_mm:=100.0 \
#      -p lift_mm:=100.0 \
#      -p place_approach_pose:="[500.0,0.0,300.0,180.0,0.0,90.0]" \
#      -p release_z_mm:=220.0 \
#      -p retreat_mm:=100.0
#
# 2. 위치와 거리 parameter의 의미는 다음과 같습니다.
#
#    normal_vel / normal_acc
#        일반 movel 이동 속도와 가속도입니다.
#
#    descent_vel / descent_acc
#        캔을 잡거나 놓기 위해 내려갈 때의 저속·저가속도입니다.
#
#    pick_approach_mm
#        토픽으로 받은 음료 Z보다 몇 mm 위로 접근할지 정합니다.
#        접근 Z = 입력 Z + pick_approach_mm
#
#    pick_descent_mm
#        접근점에서 몇 mm 내려갈지 정합니다.
#        파지 Z = 입력 Z + pick_approach_mm - pick_descent_mm
#        두 값을 같게 하면 토픽으로 받은 Z에서 캔을 잡습니다.
#
#    lift_mm
#        캔을 잡은 뒤 BASE 좌표계 +Z로 들어 올릴 거리입니다.
#
#    place_approach_pose
#        캔을 놓을 위치의 위쪽 접근 자세 [X,Y,Z,A,B,C]입니다.
#        단위는 mm/degree이고 반드시 DR_BASE 기준 TCP 자세여야 합니다.
#
#    release_z_mm
#        place_approach_pose에서 천천히 내려가 그리퍼를 열 최종 BASE Z입니다.
#        place_approach_pose의 Z보다 작은 값이어야 합니다.
#
#    retreat_mm
#        캔을 놓은 뒤 BASE 좌표계 +Z로 빠져나올 거리입니다.
#
#    move_home_on_startup / home_position / home_vel / home_acc
#        노드 시작 시 홈 자세로 먼저 이동할지와 관절 자세[J1..J6]를 정합니다.
#        기본 홈 자세는 [0, 0, 90, 0, 90, 0] degree입니다.
#        home_vel과 home_acc는 각각 deg/s, deg/s^2 단위입니다.
#
# 3. 음료별 RG2 파지 폭과 힘도 실행할 때 바꿀 수 있습니다.
#
#    ros2 run cobot_pjt beverage_test --ros-args \
#      -p gripper.cola.width_raw:=630 \
#      -p gripper.cola.force_raw:=30
#
#    width_raw 단위는 0.1 mm이므로 630은 63.0 mm입니다.
#    force_raw 단위는 0.1 N이므로 30은 3.0 N입니다.
#    tea, cola, coffee, sikhye를 각각 따로 설정할 수 있습니다.
#    open_width_raw는 잡기 전에 벌릴 폭이며 파지 폭보다 커야 합니다.
#
# 4. dry-run 로그와 작업 공간을 확인한 뒤에만 실제 동작을 활성화합니다.
#
#    -p dry_run:=false
#
# 5. 음료 선택 토픽 형식 예시입니다. 아래 숫자는 dry-run용 임의값이며,
#    실제 운전에는 사용하지 마세요. Pose는 DR_BASE 기준 TCP 자세입니다.
#
#    ros2 topic pub --once /dsr01/beverage_selection \
#      std_msgs/msg/String \
#      "{data: '콜라,400,100,250,180,0,90'}"
#
# 아래 명령은 네 음료의 복사·수정용 템플릿입니다. PICK_X 등의 글자를
# 실제로 티칭한 [X,Y,Z,A,B,C] 숫자로 바꾼 뒤 사용하세요. 잘못된 예시 좌표로
# 실제 로봇이 움직이지 않도록 일부러 숫자 placeholder를 넣지 않았습니다.
#
#    # 차
#    ros2 topic pub --once /dsr01/beverage_selection \
#      std_msgs/msg/String \
#      "{data: '차,PICK_X,PICK_Y,PICK_Z,PICK_A,PICK_B,PICK_C'}"
#
#    # 콜라
#    ros2 topic pub --once /dsr01/beverage_selection \
#      std_msgs/msg/String \
#      "{data: '콜라,PICK_X,PICK_Y,PICK_Z,PICK_A,PICK_B,PICK_C'}"
#
#    # 커피
#    ros2 topic pub --once /dsr01/beverage_selection \
#      std_msgs/msg/String \
#      "{data: '커피,PICK_X,PICK_Y,PICK_Z,PICK_A,PICK_B,PICK_C'}"
#
#    # 식혜
#    ros2 topic pub --once /dsr01/beverage_selection \
#      std_msgs/msg/String \
#      "{data: '식혜,PICK_X,PICK_Y,PICK_Z,PICK_A,PICK_B,PICK_C'}"
#
# 음료 PICK pose를 처음 정하는 가장 쉬운 방법:
#   1) 그리퍼를 연 상태로 수동/조그 운전합니다.
#   2) 핑거 사이에 캔이 들어가고, 닫으면 안정적으로 잡힐 TCP 자세로 갑니다.
#   3) get_current_posx(DR_BASE)로 [X,Y,Z,A,B,C]를 기록합니다.
#   4) 기록한 값을 위 PICK_X...PICK_C 자리에 넣습니다.
#   5) 먼저 dry_run=true로 접근·하강 계산을 확인합니다.
#
# RealSense의 optical-frame 좌표(m)를 위 토픽에 직접 넣으면 안 됩니다.
# ROS TF로 camera frame에서 robot BASE frame으로 변환하고, 단위도 mm로
# 바꾼 다음 발행해야 합니다.
# =============================================================================

import ast
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

import DR_init
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from peter.onrobot import RG


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 프로젝트에서 사용하는 네 종류의 음료와 허용할 토픽 표기입니다.
BEVERAGE_ALIASES = {
    "tea": "tea",
    "차": "tea",
    "cola": "cola",
    "coke": "cola",
    "콜라": "cola",
    "coffee": "coffee",
    "커피": "coffee",
    "sikhye": "sikhye",
    "식혜": "sikhye",
}

BEVERAGE_LABELS = {
    "tea": "차",
    "cola": "콜라",
    "coffee": "커피",
    "sikhye": "식혜",
}

# RG2 raw 단위: width=0.1 mm, force=0.1 N.
# 아래 값은 안전한 초기 테스트용 placeholder입니다. 캔 외경과 핑거 형상에
# 맞춰 각 음료의 폭을 실측하고, 실제 운전 전에 반드시 다시 조정하세요.
DEFAULT_GRIP_PROFILES = {
    "tea": (500, 200),
    "cola": (500, 200),
    "coffee": (500, 200),
    "sikhye": (500, 200),
}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


@dataclass(frozen=True)
class BeverageRequest:
    """검증이 끝난 음료 이름과 DR_BASE 기준 TCP pose를 보관합니다."""

    beverage: str
    pose: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class GripProfile:
    """한 음료에 사용할 RG2 목표 폭과 힘의 raw 값을 보관합니다."""

    width_raw: int
    force_raw: int


def _normalise_beverage(value: Any) -> str:
    """한국어·영어 음료 이름을 내부에서 사용하는 영문 ID로 통일합니다.

    지원하지 않는 이름이면 로봇이 움직이지 않도록 ``ValueError``를
    발생시킵니다.
    """
    name = str(value).strip().lower()
    try:
        return BEVERAGE_ALIASES[name]
    except KeyError as error:
        allowed = ", ".join(BEVERAGE_LABELS.values())
        raise ValueError(f"지원하지 않는 음료 '{value}'입니다: {allowed}") from error


def _validate_pose(values: Sequence[Any]) -> Tuple[float, ...]:
    """입력값이 유효한 Doosan pose 형식인지 기본 검증합니다.

    숫자 6개인지, NaN/무한대가 없는지, XYZ가 비정상적으로 큰 값이 아닌지
    확인합니다. 실제 도달 가능성이나 충돌 가능성까지 확인하지는 않습니다.
    """
    if isinstance(values, (str, bytes)) or len(values) != 6:
        raise ValueError("pose는 [x, y, z, a, b, c]의 숫자 6개여야 합니다")

    try:
        pose = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError("pose의 모든 항목은 숫자여야 합니다") from error

    if not all(math.isfinite(value) for value in pose):
        raise ValueError("pose에는 NaN 또는 무한대를 사용할 수 없습니다")

    # m 단위나 잘못된 문자열을 그대로 넣는 실수를 잡기 위한 넓은 sanity bound.
    if any(abs(value) > 2000.0 for value in pose[:3]):
        raise ValueError("XYZ가 +/-2000 mm 범위를 벗어났습니다")

    return pose


def _validate_joint_position(values: Sequence[Any]) -> Tuple[float, ...]:
    """홈 자세가 유효한 관절각 6개인지 기본 검증합니다.

    실제 M0609의 관절별 안전 제한과 충돌 여부는 컨트롤러 설정 및 현장
    환경에서 별도로 확인해야 합니다.
    """
    if isinstance(values, (str, bytes)) or len(values) != 6:
        raise ValueError("home_position은 [J1,J2,J3,J4,J5,J6] 숫자 6개여야 합니다")

    try:
        joints = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError("home_position의 모든 관절각은 숫자여야 합니다") from error

    if not all(math.isfinite(value) for value in joints):
        raise ValueError("home_position에는 NaN 또는 무한대를 사용할 수 없습니다")

    return joints


def parse_beverage_payload(payload: str) -> BeverageRequest:
    """String 토픽 내용을 검증된 ``BeverageRequest``로 변환합니다.

    JSON, Python 형식의 tuple/list, ``음료,x,y,z,a,b,c`` CSV 형식을
    지원합니다. 입력이 잘못되면 ``ValueError``를 발생시켜 동작을 막습니다.
    """
    text = payload.strip()
    if not text:
        raise ValueError("빈 음료 명령입니다")

    parsed: Any = None
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        try:
            parsed = ast.literal_eval(text)
        except (SyntaxError, ValueError):
            parsed = None

    if isinstance(parsed, dict):
        beverage = parsed.get("beverage", parsed.get("drink"))
        if beverage is None:
            raise ValueError("JSON에 'beverage' 항목이 필요합니다")

        if "pose" in parsed:
            pose_values = parsed["pose"]
        else:
            missing = [key for key in "xyzabc" if key not in parsed]
            if missing:
                raise ValueError("JSON에 'pose' 또는 x,y,z,a,b,c가 필요합니다")
            pose_values = [parsed[key] for key in "xyzabc"]
    elif isinstance(parsed, (list, tuple)):
        if len(parsed) != 7:
            raise ValueError("목록/튜플은 음료명과 pose 숫자 6개여야 합니다")
        beverage, *pose_values = parsed
    else:
        parts = [part.strip() for part in text.split(",")]
        if len(parts) != 7:
            raise ValueError(
                "명령 형식은 JSON 또는 '음료,x,y,z,a,b,c'여야 합니다"
            )
        beverage, *pose_values = parts

    return BeverageRequest(
        beverage=_normalise_beverage(beverage),
        pose=_validate_pose(pose_values),
    )


class BeverageNode(Node):
    """음료 선택 토픽을 받아 RG2 pick-and-place를 수행하는 ROS 2 노드입니다."""

    def __init__(self) -> None:
        """파라미터·토픽·작업 상태를 만들고 필요하면 장비를 연결합니다."""
        super().__init__("beverage_node", namespace=ROBOT_ID)

        # =====================================================================
        # 기본 파라미터 선언 부분
        # 위 실행 가이드의 `-p 이름:=값`으로 실행 시 기본값을 덮어쓸 수 있습니다.
        # =====================================================================
        # Task 동작 파라미터(mm, mm/s, mm/s^2). 아직 확정되지 않은 값은 여기서
        # 직접 고치기보다 ROS parameter/YAML로 조정할 수 있게 분리했습니다.
        self.declare_parameter("dry_run", False)
        self.declare_parameter("normal_vel", 60.0)
        self.declare_parameter("normal_acc", 60.0)
        self.declare_parameter("descent_vel", 10.0)
        self.declare_parameter("descent_acc", 20.0)
        self.declare_parameter("pick_approach_mm", 150.0)
        self.declare_parameter("pick_descent_mm", 150.0)
        self.declare_parameter("lift_mm", 150.0)

        # 아래 두 값은 음료 PICK 좌표가 아닙니다.
        # place_approach_pose: 트레이/배치 위치 위쪽의 안전한 접근 TCP pose
        # release_z_mm: 같은 X,Y,A,B,C를 유지하고 내려가 캔을 놓을 최종 BASE Z
        self.declare_parameter(
            "place_approach_pose", [278.016, 54.962, 174.347, 16.884, -178.725, 16.126]
        )
        self.declare_parameter("release_z_mm", 110.0)
        self.declare_parameter("retreat_mm", 150.0)
        self.declare_parameter("set_autonomous_mode", True)
        self.declare_parameter("move_home_on_startup", True)
        self.declare_parameter(
            "home_position", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        )
        self.declare_parameter("home_vel", 30.0)
        self.declare_parameter("home_acc", 30.0)

        # OnRobot RG2 설정. 30 raw = 3 N, 620 raw = 62.0 mm입니다.
        self.declare_parameter("gripper.ip", "192.168.1.1")
        self.declare_parameter("gripper.port", 502)
        self.declare_parameter("gripper.open_width_raw", 800)
        self.declare_parameter("gripper.open_force_raw", 30)
        self.declare_parameter("gripper.timeout_sec", 5.0)

        for name, (width_raw, force_raw) in DEFAULT_GRIP_PROFILES.items():
            self.declare_parameter(f"gripper.{name}.width_raw", width_raw)
            self.declare_parameter(f"gripper.{name}.force_raw", force_raw)

        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._normal_vel = float(self.get_parameter("normal_vel").value)
        self._normal_acc = float(self.get_parameter("normal_acc").value)
        self._descent_vel = float(self.get_parameter("descent_vel").value)
        self._descent_acc = float(self.get_parameter("descent_acc").value)
        self._pick_approach_mm = float(
            self.get_parameter("pick_approach_mm").value
        )
        self._pick_descent_mm = float(
            self.get_parameter("pick_descent_mm").value
        )
        self._lift_mm = float(self.get_parameter("lift_mm").value)
        self._place_approach_pose = _validate_pose(
            self.get_parameter("place_approach_pose").value
        )
        self._release_z_mm = float(self.get_parameter("release_z_mm").value)
        self._retreat_mm = float(self.get_parameter("retreat_mm").value)
        self._set_autonomous_mode = bool(
            self.get_parameter("set_autonomous_mode").value
        )
        self._move_home_on_startup = bool(
            self.get_parameter("move_home_on_startup").value
        )
        self._home_position = _validate_joint_position(
            self.get_parameter("home_position").value
        )
        self._home_vel = float(self.get_parameter("home_vel").value)
        self._home_acc = float(self.get_parameter("home_acc").value)

        self._open_width_raw = int(
            self.get_parameter("gripper.open_width_raw").value
        )
        self._open_force_raw = int(
            self.get_parameter("gripper.open_force_raw").value
        )
        self._gripper_timeout_sec = float(
            self.get_parameter("gripper.timeout_sec").value
        )
        self._grip_profiles = self._read_grip_profiles()
        self._validate_parameters()

        self._status_publisher = self.create_publisher(
            String, "beverage_status", 10
        )

        self._state_lock = threading.Lock()
        self._busy = False
        self._worker: Optional[threading.Thread] = None
        self._shutdown_requested = threading.Event()

        self._robot_node: Optional[Node] = None
        self._gripper: Optional[RG] = None
        self._movel = None
        self._movej = None
        self._posx = None
        self._posj = None
        self._set_robot_mode = None
        self._dr_base = None
        self._dr_mv_mod_abs = None
        self._dr_mv_mod_rel = None
        self._robot_mode_autonomous = None

        if self._dry_run:
            self.get_logger().warning(
                "dry_run=true: 실제 로봇과 그리퍼는 움직이지 않습니다"
            )
        else:
            self._initialise_hardware()

        # 홈 이동이 완료된 뒤에만 음료 토픽 구독을 시작합니다.
        self._move_to_home()
        self._subscription = self.create_subscription(
            String,
            "beverage_selection",
            self.beverage_callback,
            10,
        )

        if self._place_pose_is_placeholder():
            self.get_logger().warning(
                "place_approach_pose/release_z_mm가 placeholder입니다. "
                "실제 동작 전에 반드시 티칭한 BASE 좌표로 설정하세요"
            )

        self.get_logger().info(
            "음료 명령 대기 중: /dsr01/beverage_selection "
            "(차, 콜라, 커피, 식혜)"
        )

    def _read_grip_profiles(self) -> Dict[str, GripProfile]:
        """네 음료의 width/force ROS parameter를 파지 프로필로 읽습니다."""
        profiles = {}
        for name in DEFAULT_GRIP_PROFILES:
            profiles[name] = GripProfile(
                width_raw=int(
                    self.get_parameter(f"gripper.{name}.width_raw").value
                ),
                force_raw=int(
                    self.get_parameter(f"gripper.{name}.force_raw").value
                ),
            )
        return profiles

    def _validate_parameters(self) -> None:
        """시작 시 이동·그리퍼 parameter의 값과 범위를 검사합니다.

        잘못된 값으로 실제 장비가 움직이는 것을 막기 위해 하나라도 범위를
        벗어나면 노드 초기화를 중단합니다.
        """
        positive_values = {
            "normal_vel": self._normal_vel,
            "normal_acc": self._normal_acc,
            "descent_vel": self._descent_vel,
            "descent_acc": self._descent_acc,
            "pick_approach_mm": self._pick_approach_mm,
            "pick_descent_mm": self._pick_descent_mm,
            "lift_mm": self._lift_mm,
            "retreat_mm": self._retreat_mm,
            "gripper.timeout_sec": self._gripper_timeout_sec,
            "home_vel": self._home_vel,
            "home_acc": self._home_acc,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name}은 0보다 큰 유한값이어야 합니다")

        self._validate_gripper_value(
            self._open_width_raw, self._open_force_raw, "open"
        )
        for name, profile in self._grip_profiles.items():
            self._validate_gripper_value(
                profile.width_raw, profile.force_raw, name
            )
            if profile.width_raw >= self._open_width_raw:
                raise ValueError(
                    f"gripper.{name}.width_raw는 open_width_raw보다 작아야 합니다"
                )

    @staticmethod
    def _validate_gripper_value(
        width_raw: int, force_raw: int, name: str
    ) -> None:
        """RG2 목표 폭과 힘이 장치에서 허용하는 raw 범위인지 검사합니다."""
        if not 0 <= width_raw <= 1100:
            raise ValueError(f"{name} gripper width는 RG2 범위 0..1100이어야 합니다")
        if not 30 <= force_raw <= 400:
            raise ValueError(f"{name} gripper force는 RG2 범위 30..400이어야 합니다")

    def _initialise_hardware(self) -> None:
        """두산 서비스 전용 노드와 RG2 Modbus 연결을 초기화합니다.

        ``DSR_ROBOT2``는 import 시점에 ``DR_init.__dsr__node``를 사용하므로
        로봇 전용 노드를 먼저 만든 뒤 import합니다. ``dry_run=true``일 때는
        이 함수가 호출되지 않습니다.
        """
        # DSR_ROBOT2는 import 순간 DR_init.__dsr__node를 캡처합니다. 토픽
        # callback 노드와 분리된 전용 노드를 먼저 만든 뒤 import해야 합니다.
        self._robot_node = rclpy.create_node(
            "beverage_robot_client", namespace=ROBOT_ID
        )
        # 이 코드는 class method 안에 있으므로 ``DR_init.__dsr__node``로
        # 직접 대입하면 Python name mangling에 의해 다른 속성이 생깁니다.
        # setattr을 사용해 DSR_ROBOT2가 실제로 읽는 모듈 속성을 설정합니다.
        setattr(DR_init, "__dsr__node", self._robot_node)

        try:
            from DR_common2 import posj, posx
            from DSR_ROBOT2 import (
                DR_BASE,
                DR_MV_MOD_ABS,
                DR_MV_MOD_REL,
                ROBOT_MODE_AUTONOMOUS,
                movel,
                movej,
                set_robot_mode,
            )

            self._posx = posx
            self._posj = posj
            self._movel = movel
            self._movej = movej
            self._set_robot_mode = set_robot_mode
            self._dr_base = DR_BASE
            self._dr_mv_mod_abs = DR_MV_MOD_ABS
            self._dr_mv_mod_rel = DR_MV_MOD_REL
            self._robot_mode_autonomous = ROBOT_MODE_AUTONOMOUS

            self._gripper = RG(
                "rg2",
                str(self.get_parameter("gripper.ip").value),
                int(self.get_parameter("gripper.port").value),
            )
            if not self._gripper.client.is_socket_open():
                raise ConnectionError("RG2 Modbus TCP 연결에 실패했습니다")
        except Exception:
            if self._gripper is not None:
                self._gripper.close_connection()
                self._gripper = None
            if self._robot_node is not None:
                self._robot_node.destroy_node()
                self._robot_node = None
            raise

    def _set_autonomous_if_requested(self) -> None:
        """설정이 활성화된 경우 로봇을 autonomous mode로 전환합니다."""
        if self._dry_run or not self._set_autonomous_mode:
            return
        if (
            self._set_robot_mode is None
            or self._robot_mode_autonomous is None
        ):
            raise RuntimeError("Doosan robot mode API가 초기화되지 않았습니다")

        result = self._set_robot_mode(self._robot_mode_autonomous)
        if result != 0:
            raise RuntimeError("로봇을 autonomous mode로 바꾸지 못했습니다")

    def _move_to_home(self) -> None:
        """노드 시작 시 설정된 관절 홈 자세로 동기식 이동합니다.

        ``movej``는 관절 공간 이동이므로 DR_BASE 같은 task 좌표계를 사용하지
        않습니다. ``dry_run``에서는 목표 관절각만 로그로 출력합니다.
        """
        if not self._move_home_on_startup:
            self.get_logger().info("시작 홈 이동이 비활성화되어 있습니다")
            return

        self.get_logger().info(
            "시작 홈 이동: "
            f"posj({[round(value, 3) for value in self._home_position]}), "
            f"vel={self._home_vel}, acc={self._home_acc}"
        )
        if self._dry_run:
            return

        self._ensure_running()
        self._set_autonomous_if_requested()
        if self._movej is None or self._posj is None:
            raise RuntimeError("Doosan movej API가 초기화되지 않았습니다")

        result = self._movej(
            self._posj(*self._home_position),
            vel=self._home_vel,
            acc=self._home_acc,
        )
        if result != 0:
            raise RuntimeError(f"시작 홈 movej 실패(return={result})")

        self.get_logger().info("시작 홈 자세 도착 완료")

    def beverage_callback(self, msg: String) -> None:
        """음료 선택 토픽을 검증하고 비동기 작업 스레드를 시작합니다.

        이전 작업이 진행 중이면 새 명령을 거절합니다. 콜백 자체는 로봇
        동작을 기다리지 않고 worker thread에 작업을 넘깁니다.
        """
        try:
            request = parse_beverage_payload(msg.data)
        except ValueError as error:
            self.get_logger().error(f"음료 명령 거절: {error}")
            self._publish_status("rejected", error=str(error))
            return

        with self._state_lock:
            if self._busy:
                reason = "이전 음료 작업이 진행 중입니다"
                self.get_logger().warning(f"음료 명령 거절: {reason}")
                self._publish_status(
                    "rejected", beverage=request.beverage, error=reason
                )
                return
            self._busy = True

        self._publish_status("accepted", beverage=request.beverage)
        self._worker = threading.Thread(
            target=self._run_pick_and_place,
            args=(request,),
            name="beverage-pick-and-place",
            daemon=True,
        )
        self._worker.start()

    def _run_pick_and_place(self, request: BeverageRequest) -> None:
        """접근, 파지, 상승, 배치, 해제, 후퇴 순서로 작업을 실행합니다.

        각 단계는 동기식 ``movel`` 또는 그리퍼 완료 확인 후 다음 단계로
        넘어갑니다. 실패하면 남은 단계를 실행하지 않으며, 캔을 들고 있을
        가능성이 있을 때는 자동으로 그리퍼를 열지 않습니다.
        """
        holding_object = False
        label = BEVERAGE_LABELS[request.beverage]

        try:
            approach_pose = list(request.pose)
            approach_pose[2] += self._pick_approach_mm

            grasp_pose = list(approach_pose)
            grasp_pose[2] -= self._pick_descent_mm

            release_pose = list(self._place_approach_pose)
            release_pose[2] = self._release_z_mm

            for pose in (approach_pose, grasp_pose, release_pose):
                _validate_pose(pose)
            self._validate_place_configuration()

            profile = self._grip_profiles[request.beverage]
            self.get_logger().info(
                f"{label} 작업 시작: pick={list(request.pose)}, "
                f"grip_width={profile.width_raw / 10.0:.1f} mm, "
                f"grip_force={profile.force_raw / 10.0:.1f} N"
            )
            self._publish_status("working", beverage=request.beverage)

            self._ensure_running()
            self._set_autonomous_if_requested()

            # 1) 음료 위 접근점으로 이동한 뒤 그리퍼를 엽니다.
            self._move_abs(
                approach_pose,
                self._normal_vel,
                self._normal_acc,
                "pick approach",
            )
            self._command_gripper(
                self._open_width_raw,
                self._open_force_raw,
                require_grip=False,
                description="open before pick",
            )

            # 2) BASE -Z로 천천히 내려가 약한 힘으로 캔을 잡습니다.
            self._move_abs(
                grasp_pose,
                self._descent_vel,
                self._descent_acc,
                "slow pick descent",
            )
            self._command_gripper(
                profile.width_raw,
                profile.force_raw,
                require_grip=True,
                description=f"grip {label}",
            )
            holding_object = True

            # 3) 자세를 유지한 채 BASE +Z로 상승합니다.
            self._move_relative_z(
                self._lift_mm,
                self._normal_vel,
                self._normal_acc,
                "lift",
            )

            # 4) 지정된 배치 접근점으로 이동한 뒤 release Z까지 하강합니다.
            self._move_abs(
                self._place_approach_pose,
                self._normal_vel,
                self._normal_acc,
                "place approach",
            )
            self._move_abs(
                release_pose,
                self._descent_vel,
                self._descent_acc,
                "slow place descent",
            )

            # 5) 목표 Z에 도달한 뒤 놓고 BASE +Z로 빠져나옵니다.
            self._command_gripper(
                self._open_width_raw,
                self._open_force_raw,
                require_grip=False,
                description="release beverage",
            )
            holding_object = False
            self._move_relative_z(
                self._retreat_mm,
                self._normal_vel,
                self._normal_acc,
                "retreat",
            )

            self.get_logger().info(f"{label} 배치 완료")
            self._publish_status("succeeded", beverage=request.beverage)
        except Exception as error:
            # 실패 시 물체를 들고 있을 수도 있으므로 자동으로 그리퍼를 열지
            # 않습니다. 낙하 위험이 없는지 작업자가 확인해야 합니다.
            self.get_logger().error(
                f"{label} 작업 실패: {error}; holding_object={holding_object}"
            )
            self._publish_status(
                "failed",
                beverage=request.beverage,
                error=str(error),
                holding_object=holding_object,
            )
        finally:
            with self._state_lock:
                self._busy = False

    def _validate_place_configuration(self) -> None:
        """실제 동작 전에 배치 접근 pose와 release Z 설정을 검사합니다."""
        if self._dry_run:
            return
        if self._place_pose_is_placeholder():
            raise ValueError(
                "place_approach_pose와 release_z_mm를 실제 좌표로 설정해야 합니다"
            )
        if self._release_z_mm >= self._place_approach_pose[2]:
            raise ValueError(
                "release_z_mm는 place_approach_pose의 Z보다 작아야 합니다"
            )

    def _place_pose_is_placeholder(self) -> bool:
        """배치 좌표가 아직 기본값인 모두 0인지 반환합니다."""
        return (
            all(abs(value) < 1e-9 for value in self._place_approach_pose)
            and abs(self._release_z_mm) < 1e-9
        )

    def _move_abs(
        self,
        pose: Sequence[float],
        vel: float,
        acc: float,
        description: str,
    ) -> None:
        """DR_BASE 기준 절대 TCP pose로 동기식 직선 이동합니다.

        ``dry_run``에서는 계산한 pose만 로그로 출력하고 실제 ``movel``은
        호출하지 않습니다. 서비스 반환값이 0이 아니면 실패 처리합니다.
        """
        self._ensure_running()
        self.get_logger().info(
            f"{description}: posx({[round(value, 3) for value in pose]}) "
            f"vel={vel}, acc={acc}"
        )
        if self._dry_run:
            return
        if (
            self._movel is None
            or self._posx is None
            or self._dr_base is None
            or self._dr_mv_mod_abs is None
        ):
            raise RuntimeError("Doosan movel API가 초기화되지 않았습니다")

        result = self._movel(
            self._posx(*pose),
            vel=vel,
            acc=acc,
            ref=self._dr_base,
            mod=self._dr_mv_mod_abs,
        )
        if result != 0:
            raise RuntimeError(f"{description} movel 실패(return={result})")

    def _move_relative_z(
        self, distance_mm: float, vel: float, acc: float, description: str
    ) -> None:
        """TCP 방향을 유지하며 DR_BASE 기준 Z축으로 상대 직선 이동합니다.

        양수 거리는 BASE +Z 상승, 음수 거리는 BASE -Z 하강을 의미합니다.
        현재 시퀀스에서는 파지 후 상승과 배치 후 후퇴에 사용합니다.
        """
        self._ensure_running()
        self.get_logger().info(
            f"{description}: BASE relative Z={distance_mm:.3f} mm "
            f"vel={vel}, acc={acc}"
        )
        if self._dry_run:
            return
        if (
            self._movel is None
            or self._posx is None
            or self._dr_base is None
            or self._dr_mv_mod_rel is None
        ):
            raise RuntimeError("Doosan relative movel API가 초기화되지 않았습니다")

        result = self._movel(
            self._posx(0.0, 0.0, distance_mm, 0.0, 0.0, 0.0),
            vel=vel,
            acc=acc,
            ref=self._dr_base,
            mod=self._dr_mv_mod_rel,
        )
        if result != 0:
            raise RuntimeError(f"{description} movel 실패(return={result})")

    def _command_gripper(
        self,
        width_raw: int,
        force_raw: int,
        require_grip: bool,
        description: str,
    ) -> None:
        """RG2에 폭·힘 명령을 보내고 완료 또는 파지 감지를 기다립니다.

        ``require_grip=True``이면 busy 해제 후 grip-detected 비트를
        확인합니다. ``False``이면 실제 폭이 목표의 +/-3 mm인지 검사합니다.
        제한 시간 안에 끝나지 않으면 예외를 발생시킵니다.
        """
        self._ensure_running()
        self.get_logger().info(
            f"{description}: width={width_raw / 10.0:.1f} mm, "
            f"force={force_raw / 10.0:.1f} N"
        )
        if self._dry_run:
            return
        if self._gripper is None:
            raise RuntimeError("RG2가 초기화되지 않았습니다")

        self._gripper.move_gripper(width_raw, force_raw)
        # Modbus 명령이 busy 상태에 반영될 시간을 잠깐 줍니다.
        time.sleep(0.2)
        deadline = time.monotonic() + self._gripper_timeout_sec

        while time.monotonic() < deadline:
            self._ensure_running()
            status = self._gripper.get_status()
            if status[0] == 0:
                if require_grip:
                    if status[1] == 1:
                        return
                    raise RuntimeError("RG2가 물체 파지를 감지하지 못했습니다")

                actual_width = self._gripper.get_width_with_offset()
                target_width = width_raw / 10.0
                if abs(actual_width - target_width) <= 3.0:
                    return
                raise RuntimeError(
                    "RG2 열림 폭이 목표에 도달하지 못했습니다: "
                    f"target={target_width:.1f} mm, "
                    f"actual={actual_width:.1f} mm"
                )
            time.sleep(0.1)

        raise TimeoutError(f"RG2 {description} 동작 시간이 초과되었습니다")

    def _ensure_running(self) -> None:
        """종료 요청 후에는 다음 장비 동작을 시작하지 않도록 막습니다."""
        if self._shutdown_requested.is_set():
            raise RuntimeError("노드 종료 요청으로 작업을 중단합니다")

    def _publish_status(self, state: str, **fields: Any) -> None:
        """작업 상태를 JSON String으로 beverage_status 토픽에 발행합니다."""
        payload = {"state": state, **fields}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(msg)

    def close(self) -> None:
        """작업 스레드를 정리하고 RG2 및 로봇 노드 연결을 종료합니다."""
        self._shutdown_requested.set()
        worker = self._worker
        if worker is not None and worker.is_alive():
            self.get_logger().warning(
                "진행 중 작업의 현재 단계가 끝나기를 최대 5초간 기다립니다"
            )
            worker.join(timeout=5.0)

        if worker is not None and worker.is_alive():
            self.get_logger().error(
                "작업 스레드가 아직 실행 중입니다. 로봇 상태와 비상정지를 "
                "직접 확인하세요"
            )
            return

        if self._gripper is not None:
            self._gripper.close_connection()
            self._gripper = None
        if self._robot_node is not None:
            self._robot_node.destroy_node()
            self._robot_node = None


def main(args=None) -> None:
    """ROS 2를 초기화하고 음료 노드를 명시적 executor에서 실행합니다.

    토픽 노드는 ``SingleThreadedExecutor``가 처리하고, DSR 동기 서비스는
    내부 global executor를 사용하도록 분리해 동시 spin 충돌을 방지합니다.
    """
    rclpy.init(args=args)
    node: Optional[BeverageNode] = None
    executor: Optional[SingleThreadedExecutor] = None

    try:
        node = BeverageNode()

        # DSR_ROBOT2의 동기 service 함수는 rclpy global executor를 내부에서
        # 사용합니다. 토픽 노드는 별도 executor로 돌려 두 executor가 worker
        # thread에서 동시에 spin하는 충돌을 피합니다.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
