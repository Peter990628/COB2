# go_home.py

import rclpy
import DR_init
import tkinter as tk


# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
ON, OFF = 1, 0

# Initialize DR_init with robot parameters
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        "dsr_rokey_basic_py",
        namespace=ROBOT_ID,
    )
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import movej, DR_BASE
        from DR_common2 import posj

        node.get_logger().info("Moving to home position...")

        home_position = posj(0, 0, 90, 0, 90, 0)
        movej(home_position, VELOCITY, ACC, DR_BASE)

        node.get_logger().info("Home position reached.")

    except ImportError as error:
        node.get_logger().error(
            f"Error importing robot module: {error}"
        )
    except Exception as error:
        node.get_logger().error(
            f"Failed to move home: {error}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    


if __name__ == "__main__":
    main()