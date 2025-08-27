import dairlib.lcmt_franka_target_joint_position
import lcm
import numpy as np
import time

WAIT_TIME_BETWEEN_TARGETS = 10.0


def main():
    channel = "FRANKA_TARGET_JOINT_POSITION"
    publisher = lcm.LCM()

    target_joint_positions = np.array(
        [
            [
                2.11040422,
                1.49661226,
                -2.19194702,
                -2.25278088,
                2.16248473,
                0.97698776,
                2.21014229,
            ],
            [
                1.71497384,
                1.49221002,
                -2.0323272,
                -1.87310179,
                2.51143497,
                1.29020946,
                2.39898683,
            ],
            [
                1.35857796,
                1.49284616,
                -1.95374991,
                -1.56879378,
                2.51031323,
                1.71089151,
                2.61750719,
            ],
            [
                1.06488494,
                1.49563892,
                -1.83291149,
                -1.40618489,
                2.50079103,
                1.98715143,
                2.72593251,
            ],
            [
                0.78576641,
                1.4958888,
                -1.74013677,
                -1.19640547,
                2.50141325,
                2.37673133,
                2.88127821,
            ],
            [
                0.75839748,
                1.49320679,
                -1.74868152,
                -1.47016618,
                2.504014,
                2.37726253,
                2.98508856,
            ],
            [
                0.51181142,
                1.5171782,
                -1.83157782,
                -1.33855712,
                1.51882982,
                2.15502102,
                -2.31769613,
            ],
            [
                0.59774911,
                1.64082105,
                -1.91301015,
                -1.85192501,
                1.36122806,
                1.97927371,
                -2.16064042,
            ],
            [
                0.89569848,
                1.44325207,
                -1.97668558,
                -2.5125021,
                0.50937241,
                1.8012832,
                -1.38368678,
            ],
            [
                2.47403703,
                1.04390667,
                -2.23687388,
                -2.52138586,
                0.90766719,
                0.9410931,
                1.00877264,
            ],
            [2.191, 1.1, -1.33, -2.22, 1.30, 2.02, 0.08],
        ]
    )

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
