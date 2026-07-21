# gripper_test.py
import time

from peter.onrobot import RG


def main(args=None):
    gripper = RG("rg2", "192.168.1.1", 502)

    try:
        # 너비 62mm, 힘 20 N
        gripper.move_gripper(width_val=620, force_val=200)
        print("Gripper moved to width 62mm with force 20N")
        time.sleep(5)

        # 너비 20 mm, 힘 40 N
        gripper.move_gripper(width_val=20, force_val=400)
        print("Gripper moved to width 20mm with force 40N")
        time.sleep(2)
    finally:
        gripper.close_connection()


if __name__ == "__main__":
    main()