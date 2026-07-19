# gripper_test.py
import time

from peter.onrobot import RG


def main(args=None):
    gripper = RG("rg2", "192.168.1.1", 502)

    try:
        # 너비 80 mm, 힘 20 N
        gripper.move_gripper(width_val=500, force_val=200)
        time.sleep(2)

        # 너비 30 mm, 힘 20 N
        gripper.move_gripper(width_val=10, force_val=400)
        time.sleep(2)
    finally:
        gripper.close_connection()


if __name__ == "__main__":
    main()