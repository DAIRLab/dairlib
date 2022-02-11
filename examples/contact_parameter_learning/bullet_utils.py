
joint_info_return = ['joint_index', 'joint_name', 'joint_type', 'q_index',
                     'v_index', 'flags', 'damping', 'friction',
                     'joint_limit_lower', 'joint_limit_upper',
                     'force_limit', 'vel_limit', 'link_name', 'joint_axis',
                     'joint_pos_in_parent', 'joint_quat_in_parent',
                     'parent_frame_idx']

joint_info_map = {joint_info_return[i]: i for i in range(len(joint_info_return))}