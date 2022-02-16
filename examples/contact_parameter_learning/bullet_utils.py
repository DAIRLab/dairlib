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
ik_constants = {
    'left_thigh_to_rod_end': np.array([0, 0, 0.045]),
    'right_thigh_to_rod_end': np.array([0, 0, -0.045]),
    'heel_spring_to_rod_end': np.array([.11877, -.01, 0])
}


def achilles_ik(plant, context, x):
    plant.SetPositionsAndVelocities(context, x)
    rod_joint_angles = {'left': {}, 'right': {}}

    for side in ['left', 'right']:
        hip_connection_point = ik_constants[side + '_thigh_to_rod_end']
        heel_connection_point = plant.CalcPointsPositions(
            context,
            plant.GetBodyByName('heel_spring_' + side).body_frame(),
            ik_constants['heel_spring_to_rod_end'],
            plant.GetBodyByName('thigh_'+side).body_frame()).ravel()

        achilles_rod_in_thigh_frame = heel_connection_point - hip_connection_point
        u = achilles_rod_in_thigh_frame / \
            np.linalg.norm(achilles_rod_in_thigh_frame)

        pitch_angle = np.arcsin(u[1])
        roll_angle = np.arccos(u[0] / np.cos(pitch_angle))
        
        rod_joint_angles[side]['pitch'] = pitch_angle
        rod_joint_angles[side]['roll'] = roll_angle

    return rod_joint_angles