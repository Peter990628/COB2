"""검출된 음료 위치로 로봇 TCP를 한 번 이동시키는 ROS 2 노드."""
# move_to_bev
import math
import time

import rclpy
from checkin_interfaces.msg import BeveragePosition
from hotel_vision.gripper.onrobot import RG
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# 실행 시 처음 이동할 관절 자세 [J1, J2, J3, J4, J5, J6] (deg)
INITIAL_JOINT_POSE = [20.0, 25.0, 85.0, -50.0, 100.0, 210.0]

# 캔 위쪽 접근점까지 이동할 때 사용할 TCP 자세 [A, B, C] (deg)
TARGET_ABC = [83.411, -122.476, -91.566]

# 캔 좌표를 기준으로 한 BASE 좌표계 이동량 (mm)
ABOVE_CAN_Z_OFFSET_MM = 50.0
BASE_Y_ADJUST_MM = -25.0
BASE_Z_DESCENT_MM = -25.0

# TOOL Z축이 BASE -Z 방향을 보도록 만드는 수직 파지 자세 (deg)
VERTICAL_TOOL_ABC = [0.0, -180.0, -90.0]

JOINT_VELOCITY = 30.0
JOINT_ACCELERATION = 30.0
LINEAR_VELOCITY = 20.0
LINEAR_ACCELERATION = 20.0
FINE_LINEAR_VELOCITY = 5.0
FINE_LINEAR_ACCELERATION = 5.0

# OnRobot RG2 raw 단위: width=0.1 mm, force=0.1 N
GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502
GRIPPER_OPEN_WIDTH_RAW = 800
GRIPPER_GRIP_WIDTH_RAW = 500
GRIPPER_FORCE_RAW = 200
GRIPPER_TIMEOUT_SEC = 5.0

BEVERAGE_POSITION_TOPIC = "/checkin/beverage_position"


def command_gripper(
    gripper: RG,
    logger,
    width_raw: int,
    force_raw: int,
    require_grip: bool,
    description: str,
) -> None:
    """RG2 명령 후 동작 완료 및 필요 시 물체 파지를 확인한다."""
    logger.info(
        f"{description}: width={width_raw / 10.0:.1f} mm, "
        f"force={force_raw / 10.0:.1f} N"
    )
    gripper.move_gripper(
        width_val=width_raw,
        force_val=force_raw,
    )

    # Modbus 명령이 RG2의 busy 상태에 반영될 시간을 준다.
    time.sleep(0.2)
    deadline = time.monotonic() + GRIPPER_TIMEOUT_SEC

    while time.monotonic() < deadline:
        status = gripper.get_status()

        if status[0] == 0:
            if require_grip and status[1] != 1:
                raise RuntimeError(
                    "RG2가 캔 파지를 감지하지 못했습니다."
                )

            logger.info(f"{description} 완료")
            return

        time.sleep(0.1)

    raise TimeoutError(
        f"{description} 동작이 {GRIPPER_TIMEOUT_SEC:.1f}초 안에 "
        "완료되지 않았습니다."
    )


class BeveragePositionReceiver(Node):
    """검출 노드가 발행한 첫 번째 유효 BASE 좌표를 저장한다."""

    def __init__(self) -> None:
        super().__init__("move_to_bev_receiver")

        self.received_position = None

        # detector와 같은 QoS를 사용한다. TRANSIENT_LOCAL 덕분에 detector가
        # 먼저 한 번 발행했더라도 살아 있는 publisher의 마지막 값을 받을 수 있다.
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.position_subscription = self.create_subscription(
            BeveragePosition,
            BEVERAGE_POSITION_TOPIC,
            self._position_callback,
            qos_profile,
        )

    def _position_callback(self, msg: BeveragePosition) -> None:
        """첫 번째 정상 좌표만 저장하고 로봇 이동은 main에서 수행한다."""
        if self.received_position is not None:
            return

        xyz = (float(msg.base_x), float(msg.base_y), float(msg.base_z))

        if not all(math.isfinite(value) for value in xyz):
            self.get_logger().warning(
                f"유효하지 않은 음료 좌표를 무시합니다: {xyz}"
            )
            return

        self.received_position = {
            "class_name": msg.class_name,
            "confidence": float(msg.confidence),
            "x": xyz[0],
            "y": xyz[1],
            "z": xyz[2],
        }

        self.get_logger().info(
            "음료 좌표를 받았습니다: "
            f"class={msg.class_name}, confidence={msg.confidence:.3f}, "
            f"BASE XYZ=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) mm"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    robot_node = None
    receiver_node = None
    gripper = None

    try:
        # DSR_ROBOT2를 import하기 전에 반드시 DR_init에 ROS 노드를 넣어야 한다.
        # 순서를 바꾸면 g_node가 None이 되어 create_client 오류가 발생한다.
        robot_node = rclpy.create_node(
            "move_to_bev_robot",
            namespace=ROBOT_ID,
        )
        DR_init.__dsr__node = robot_node

        from DSR_ROBOT2 import (
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            movej,
            movel,
            posj,
            posx,
        )

        # 구독 노드와 DSR 서비스 노드를 분리한다. 콜백은 좌표만 저장하며,
        # DSR의 동기 서비스 호출(movej/movel)은 콜백 밖에서 실행한다.
        receiver_node = BeveragePositionReceiver()

        robot_node.get_logger().info(
            f"초기 관절 자세로 이동합니다: {INITIAL_JOINT_POSE}"
        )
        movej_result = movej(
            posj(INITIAL_JOINT_POSE),
            vel=JOINT_VELOCITY,
            acc=JOINT_ACCELERATION,
        )
        if movej_result != 0:
            raise RuntimeError(
                f"초기 movej에 실패했습니다. 반환값: {movej_result}"
            )

        # 캔으로 접근하기 전에 그리퍼를 충분히 연다.
        gripper = RG("rg2", GRIPPER_IP, GRIPPER_PORT)
        command_gripper(
            gripper,
            robot_node.get_logger(),
            GRIPPER_OPEN_WIDTH_RAW,
            GRIPPER_FORCE_RAW,
            require_grip=False,
            description="RG2 열기",
        )

        receiver_node.get_logger().info(
            f"{BEVERAGE_POSITION_TOPIC} 토픽을 기다립니다."
        )

        # detector가 발행한 유효 좌표 하나를 받을 때까지 기다린다.
        while rclpy.ok() and receiver_node.received_position is None:
            rclpy.spin_once(receiver_node, timeout_sec=0.1)

        if not rclpy.ok():
            return

        received = receiver_node.received_position

        # 1. 검출 좌표보다 BASE +Z 방향으로 10 mm 높은 위치로 이동한다.
        above_can_pose = posx(
            [
                received["x"],
                received["y"],
                received["z"] + ABOVE_CAN_Z_OFFSET_MM,
                *TARGET_ABC,
            ]
        )

        robot_node.get_logger().info(
            "캔 위쪽 접근 pose (BASE): "
            f"{list(above_can_pose)}"
        )
        robot_node.get_logger().info(
            f"검출 좌표에서 BASE +Z {ABOVE_CAN_Z_OFFSET_MM:.1f} mm "
            "위치로 이동합니다."
        )

        above_move_result = movel(
            above_can_pose,
            vel=LINEAR_VELOCITY,
            acc=LINEAR_ACCELERATION,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        if above_move_result != 0:
            raise RuntimeError(
                "캔 위쪽 접근 movel에 실패했습니다. "
                f"반환값: {above_move_result}"
            )

        # 2. 같은 XYZ에서 TOOL이 BASE -Z 방향을 보도록 수직 정렬한다.
        vertical_pose = posx(
            [
                received["x"],
                received["y"],
                received["z"] + ABOVE_CAN_Z_OFFSET_MM,
                *VERTICAL_TOOL_ABC,
            ]
        )

        robot_node.get_logger().info(
            "TOOL 수직 정렬 pose (BASE): "
            f"{list(vertical_pose)}"
        )

        align_result = movel(
            vertical_pose,
            vel=FINE_LINEAR_VELOCITY,
            acc=FINE_LINEAR_ACCELERATION,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        if align_result != 0:
            raise RuntimeError(
                "TOOL 수직 정렬 movel에 실패했습니다. "
                f"반환값: {align_result}"
            )

        # 3. 수직 자세를 유지한 채 BASE -Y 방향으로 25 mm 이동한다.
        y_adjust_result = movel(
            posx([0.0, BASE_Y_ADJUST_MM, 0.0, 0.0, 0.0, 0.0]),
            vel=FINE_LINEAR_VELOCITY,
            acc=FINE_LINEAR_ACCELERATION,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
        )
        if y_adjust_result != 0:
            raise RuntimeError(
                "BASE -Y 보정 movel에 실패했습니다. "
                f"반환값: {y_adjust_result}"
            )

        robot_node.get_logger().info(
            f"BASE Y 방향 {BASE_Y_ADJUST_MM:.1f} mm 이동 완료"
        )

        # 4. BASE -Z 방향으로 25 mm 천천히 내려간다.
        z_descent_result = movel(
            posx([0.0, 0.0, BASE_Z_DESCENT_MM, 0.0, 0.0, 0.0]),
            vel=FINE_LINEAR_VELOCITY,
            acc=FINE_LINEAR_ACCELERATION,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
        )
        if z_descent_result != 0:
            raise RuntimeError(
                "BASE -Z 하강 movel에 실패했습니다. "
                f"반환값: {z_descent_result}"
            )

        robot_node.get_logger().info(
            f"BASE Z 방향 {BASE_Z_DESCENT_MM:.1f} mm 이동 완료"
        )

        # 5. 캔 지름(약 66 mm)보다 조금 작은 50 mm, 20 N으로 파지한다.
        command_gripper(
            gripper,
            robot_node.get_logger(),
            GRIPPER_GRIP_WIDTH_RAW,
            GRIPPER_FORCE_RAW,
            require_grip=True,
            description="캔 파지",
        )

        robot_node.get_logger().info(
            "상부 접근 및 캔 파지를 완료했습니다."
        )

    except KeyboardInterrupt:
        if robot_node is not None:
            robot_node.get_logger().info("사용자 요청으로 종료합니다.")

    except Exception as error:
        if robot_node is not None:
            robot_node.get_logger().error(f"move_to_bev 오류: {error}")
        else:
            print(f"move_to_bev 노드 생성 실패: {error}")

    finally:
        if gripper is not None:
            gripper.close_connection()

        if receiver_node is not None:
            receiver_node.destroy_node()

        if robot_node is not None:
            robot_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
