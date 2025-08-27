import dairlib.lcmt_franka_target_joint_position
import lcm
import numpy as np
import time

WAIT_TIME_BETWEEN_TARGETS = 10.0


def main():
    channel = "FRANKA_TARGET_JOINT_POSITION"
    publisher = lcm.LCM()

    target_joint_positions = [
        np.array([1.5, 1.1, -1.33, -2.22, 1.30, 2.02, 0.08]),
        np.array([2.191, 1.1, -1.33, -2.22, 1.30, 2.02, 0.08]),
    ]

    for joint_pos in target_joint_positions:
        print(
            f"Moving to joint positions: {joint_pos}, wait for {WAIT_TIME_BETWEEN_TARGETS} seconds"
        )
        msg = dairlib.lcmt_franka_target_joint_position()
        msg.target_joint_position = joint_pos
        msg.utime = int(time.time() * 1e6)
        publisher.publish(channel, msg.encode())
        time.sleep(WAIT_TIME_BETWEEN_TARGETS)

    print(
        "Finished moving to all target joint positions, the script will be terminated now"
    )


if __name__ == "__main__":
    main()
