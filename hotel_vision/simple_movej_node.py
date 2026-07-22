#!/usr/bin/env python3
# simple_movej_node


import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "simple_movej_node",
        namespace=ROBOT_ID
    )

    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej,
        movel,
        posj,
        posx,
        get_current_posx,
    )

    # 1. 원래 사용하던 관절 자세로 이동
    movej(
        posj([20, 25, 85, -50, 100, 210]),
        vel=10,
        acc=10
    )

    # 2. movej 완료 후 현재 TCP 자세 확인
    current_pos, solution_space = get_current_posx()

    node.get_logger().info(
        f"현재 TCP Pose: {current_pos}"
    )
    node.get_logger().info(
        f"Solution Space: {solution_space}"
    )

    # 3. 목표 XYZ로 직선 이동
    # 회전 자세는 현재 TCP 자세를 그대로 사용
    movel(
        posx([
            333.33,
            -456.88,
            50.38,
            current_pos[3],
            current_pos[4],
            current_pos[5],
        ]),
        vel=10,
        acc=10
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()