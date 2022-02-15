import pdb

import numpy as np

joint_info_return = ['joint_index', 'joint_name', 'joint_type', 'q_index',
                     'v_index', 'flags', 'damping', 'friction',
                     'joint_limit_lower', 'joint_limit_upper',
                     'force_limit', 'vel_limit', 'link_name', 'joint_axis',
                     'joint_pos_in_parent', 'joint_quat_in_parent',
                     'parent_frame_idx']

joint_info_map = {joint_info_return[i]: i for i in range(len(joint_info_return))}

# All quantities are expressed here in the body frame of the first link in the
# key name
ik_contants = {
    'left_thigh_to_rod_end': np.array([0, 0, 0.045]),
    'heel_spring_to_rod_end': np.array([.11877, -.01, 0]),
    'achilles_len': 0.5012
}


def left_leg_achilles_ik(plant, context, name_to_pos_map, name_to_vel_map, x):
    plant.SetPositionsAndVelocities(context, x)
    hip_connection_point = ik_contants['left_thigh_to_rod_end']
    heel_connection_point = plant.CalcPointsPositions(
        context,
        plant.GetBodyByName("heel_spring_left").body_frame(),
        ik_contants['heel_spring_to_rod_end'],
        plant.GetBodyByName("thigh_left").body_frame()).ravel()

    achilles_rod_in_thigh_frame = heel_connection_point - hip_connection_point
    achilles_rod_in_thigh_xy = np.array([achilles_rod_in_thigh_frame[0],
                                         achilles_rod_in_thigh_frame[1], 0])

    pitch_angle = -np.arccos(achilles_rod_in_thigh_xy[0] /
                             np.linalg.norm(achilles_rod_in_thigh_xy))

    roll_angle = -np.arccos(np.dot(achilles_rod_in_thigh_frame,
                                   achilles_rod_in_thigh_xy) /
                            (np.linalg.norm(achilles_rod_in_thigh_frame) *
                            np.linalg.norm(achilles_rod_in_thigh_xy)))

    return roll_angle, pitch_angle

