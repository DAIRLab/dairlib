import dairlib
import lcm
import time


#Default values
joint_names = [
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

position_names = [
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


# Set of gains with which COM is within support polygon when we lower the hoist
joint_default = [-0.01,.01,0,0,0.55,0.55,-1.5,-1.5,-1.8,-1.8]
kp_default = [80,80,50,50,50,50,50,50,10,10]
kd_default = [1,1,1,1,1,1,2,2,1,1]


def publish_pd(lc, ramp_up_time):
    msg = dairlib.lcmt_pd_config()
    msg.num_joints = 10
    msg.joint_names = joint_names
    msg.desired_position = joint_default
    msg.desired_velocity = [0,0,0,0,0,0,0,0,0,0]

    for i in range(100):
        # ramp up the gains for 5 seconds
        msg.timestamp = int(time.time() * 1e6)
        msg.kp = [kp_default[j] * i / 100 for j in range(len(joint_names))]
        msg.kd = [kd_default[j] * i / 100 for j in range(len(joint_names))]
        lc.publish("PD_CONFIG", msg.encode())
        time.sleep(ramp_up_time / 100.0)


def main():
    lc = lcm.LCM()
    publish_pd(lc, 2.0)


if __name__ == "__main__":
    main()
