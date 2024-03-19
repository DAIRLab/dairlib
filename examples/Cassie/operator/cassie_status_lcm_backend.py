import lcm
import time
from dairlib import (lcmt_robot_output, lcmt_input_supervisor_status,
                     lcmt_pd_config, lcmt_target_standing_height)


class PDPublisher:
    def __init__(self, lc: lcm.LCM):
        self.lc = lc
        self.joint_default = \
            [-0.01, .01, 0, 0, 0.55, 0.55, -1.5, -1.5, -1.8, -1.8]
        self.kp = [80, 80, 50, 50, 50, 50, 50, 50, 10, 10]
        self.kd = [1, 1, 1, 1, 1, 1, 2, 2, 1, 1]
        self.joint_names = [
            "hip_roll_left_motor",
            "hip_roll_right_motor",
            "hip_yaw_left_motor",
            "hip_yaw_right_motor",
            "hip_pitch_left_motor",
            "hip_pitch_right_motor",
            "knee_left_motor",
            "knee_right_motor",
            "toe_left_motor",
            "toe_right_motor"]

        self.position_names = [
            "hip_roll_left",
            "hip_roll_right",
            "hip_yaw_left",
            "hip_yaw_right",
            "hip_pitch_left",
            "hip_pitch_right",
            "knee_left",
            "knee_right",
            "toe_left",
            "toe_right"]
        self.ramp_up_time = 2.0

    def publish(self):
        msg = lcmt_pd_config()
        msg.num_joints = 10
        msg.joint_names = self.joint_names
        msg.desired_velocity = [0] * len(self.joint_names)

        for i in range(100):
            # ramp up the gains for 5 seconds
            msg.timestamp = int(time.time() * 1e6)
            msg.kp = [kp * i / 99.0 for kp in self.kp]
            msg.kd = [kd * i / 99.0 for kd in self.kp]
            msg.desired_position = self.joint_default

            self.lc.publish("PD_CONFIG", msg.encode())
            time.sleep(self.ramp_up_time / 100.0)


class HeightPublisher:
    def __init__(self, lc: lcm.LCM):
        self.lc = lc

    def publish(self, height: float):
        height_msg = lcmt_target_standing_height()
        height_msg.timestamp = int(time.time() * 1e6)
        height_msg.target_height = height
        self.lc.publish("TARGET_HEIGHT", height_msg.encode())


class StatusSubscriber:
    def __init__(self, lc: lcm.LCM, channel: str):
        self.lc = lc
        self.channel = channel
        self.status_text = ''
        self.subscription = self.lc.subscribe(
            channel,
            self.status_msg_callback
        )

    def status_msg_callback(self, _, msg):
        msg = lcmt_input_supervisor_status.decode(msg)
        shutdown = msg.shutdown

        status_list = [
            f"active channel: {msg.active_channel}", f"shutdown: {shutdown}"
        ]
        status_list += [
            f"{name}: {state}" for name, state in
            zip(msg.status_names, msg.status_states)
        ]
        status_list += ['', f"Controller Time: {msg.utime * 1e-6:.3f}"]
        self.status_text = '\n'.join(status_list)
