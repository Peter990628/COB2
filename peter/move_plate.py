# move_plate
import rclpy
import DR_init
from peter.onrobot import RG

# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

# Initialize DR_init with robot parameters
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(
        "move_plate",
        namespace=ROBOT_ID,
    )
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import movej, movel, DR_BASE, wait, DR_MV_MOD_ABS,DR_MV_MOD_REL
        from DR_common2 import posj, posx
        gripper = RG("rg2", "192.168.1.1", 502)

        gripper.open_gripper()
        gripper.close_gripper()

        grap_plate_position = posj(-1.90, -15.67,131.03, 0.06, 64.64, -1.93)

        home_position = posj(0, 0, 90, 0, 90, 0)
        movej(home_position, VELOCITY, ACC, DR_BASE)

        gripper.open_gripper()
        movej(grap_plate_position, VELOCITY, ACC)
        node.get_logger().info("grap_plate_position reached.")
        gripper.close_gripper()
        wait(1)
        movel(posx(508.63,0,0,0,0,0,), VELOCITY, ACC, ref = DR_BASE ,  mod = DR_MV_MOD_REL )
        gripper.open_gripper()
        node.get_logger().info("Plate moved successfully.")
        gripper.close_connection()

    except ImportError as error:
        node.get_logger().error(
            f"Error importing robot module: {error}"
        )
    except Exception as error:
        node.get_logger().error(
            f"Failed to move grap_plate_position: {error}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    


if __name__ == "__main__":
    main()