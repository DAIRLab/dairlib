import numpy as np
from scipy.spatial.transform import Rotation as R

import dairlib

def init_robot_output():
  robot_output_message = dairlib.lcmt_robot_output()
  # Although this is different than the ordering of MBP, state receiver will rearrange accordingly
  robot_output_message.position_names = 23*['']
  robot_output_message.position = 23 * [0]
  robot_output_message.position_names[0] = 'base_x'
  robot_output_message.position_names[1] = 'base_y'
  robot_output_message.position_names[2] = 'base_z'
  robot_output_message.position_names[3] = 'base_qw'
  robot_output_message.position_names[4] = 'base_qx'
  robot_output_message.position_names[5] = 'base_qy'
  robot_output_message.position_names[6] = 'base_qz'

  robot_output_message.velocity_names = 22*['']
  robot_output_message.velocity = 23 * [0]
  robot_output_message.velocity_names[0] = 'base_vx'
  robot_output_message.velocity_names[1] = 'base_vy'
  robot_output_message.velocity_names[2] = 'base_vz'
  robot_output_message.velocity_names[3] = 'base_wx'
  robot_output_message.velocity_names[4] = 'base_wy'
  robot_output_message.velocity_names[5] = 'base_wz'

  qoffset = 7
  voffset = 6

  robot_output_message.num_positions = 16 + qoffset
  robot_output_message.num_velocities = 16 + voffset

  robot_output_message.position_names[qoffset] = 'hip_roll_left'
  robot_output_message.position_names[qoffset + 1] = 'hip_yaw_left'
  robot_output_message.position_names[qoffset + 2] = 'hip_pitch_left'
  robot_output_message.position_names[qoffset + 3] = 'knee_left'
  robot_output_message.position_names[qoffset + 4] = 'toe_left'
  robot_output_message.position_names[qoffset + 5] = 'knee_joint_left'
  robot_output_message.position_names[qoffset + 6] = 'ankle_joint_left'
  robot_output_message.position_names[qoffset + 7] = 'ankle_spring_joint_left'
  robot_output_message.position_names[qoffset + 8] = 'hip_roll_right'
  robot_output_message.position_names[qoffset + 9] = 'hip_yaw_right'
  robot_output_message.position_names[qoffset + 10] = 'hip_pitch_right'
  robot_output_message.position_names[qoffset + 11] = 'knee_right'
  robot_output_message.position_names[qoffset + 12] = 'toe_right'
  robot_output_message.position_names[qoffset + 13] = 'knee_joint_right'
  robot_output_message.position_names[qoffset + 14] = 'ankle_joint_right'
  robot_output_message.position_names[qoffset + 15] = 'ankle_spring_joint_right'

  robot_output_message.velocity_names[voffset] = 'hip_roll_leftdot'
  robot_output_message.velocity_names[voffset + 1] = 'hip_yaw_leftdot'
  robot_output_message.velocity_names[voffset + 2] = 'hip_pitch_leftdot'
  robot_output_message.velocity_names[voffset + 3] = 'knee_leftdot'
  robot_output_message.velocity_names[voffset + 4] = 'toe_leftdot'
  robot_output_message.velocity_names[voffset + 5] = 'knee_joint_leftdot'
  robot_output_message.velocity_names[voffset + 6] = 'ankle_joint_leftdot'
  robot_output_message.velocity_names[voffset + 7] = 'ankle_spring_joint_leftdot'
  robot_output_message.velocity_names[voffset + 8] = 'hip_roll_rightdot'
  robot_output_message.velocity_names[voffset + 9] = 'hip_yaw_rightdot'
  robot_output_message.velocity_names[voffset + 10] = 'hip_pitch_rightdot'
  robot_output_message.velocity_names[voffset + 11] = 'knee_rightdot'
  robot_output_message.velocity_names[voffset + 12] = 'toe_rightdot'
  robot_output_message.velocity_names[voffset + 13] = 'knee_joint_rightdot'
  robot_output_message.velocity_names[voffset + 14] = 'ankle_joint_rightdot'
  robot_output_message.velocity_names[voffset + 15] = 'ankle_spring_joint_rightdot'

  robot_output_message.num_efforts = 10
  robot_output_message.effort = 10 * [0]
  robot_output_message.effort_names = 10 * ['']
  robot_output_message.effort_names[0] = 'hip_roll_left_motor'
  robot_output_message.effort_names[1] = 'hip_yaw_left_motor'
  robot_output_message.effort_names[2] = 'hip_pitch_left_motor'
  robot_output_message.effort_names[3] = 'knee_left_motor'
  robot_output_message.effort_names[4] = 'toe_left_motor'
  robot_output_message.effort_names[5] = 'hip_roll_right_motor'
  robot_output_message.effort_names[6] = 'hip_yaw_right_motor'
  robot_output_message.effort_names[7] = 'hip_pitch_right_motor'
  robot_output_message.effort_names[8] = 'knee_right_motor'
  robot_output_message.effort_names[9] = 'toe_right_motor'
  return robot_output_message

def init_cassie_out():
  return

def pack_robot_output(robot_output, q, v, u, t):
  q = np.array(q)
  v = np.array(v)
  u = np.array(u)
  # import pdb; pdb.set_trace()
  robot_output.utime = int(t * 1e6)
  # If we are not in a floating base, need to skip the first 7 postions
  # (x,y,z,quat) and the first 6 velocities (linear and ang. velocity)
  qoffset = 0
  voffset = 0
  floating_base = True
  if floating_base:
    qoffset = 7
    voffset = 6

  # Get positions

  # See cassiemujoco.h cassie_sim_qpos for ordering of q
  # q starts with floating base coordinates and left hip,  identical to
  # the joint ordering chosen in cassiesim.c for the message
  # The offsetting here is to (1) get the proper length and (2) skip floating
  # base coordinates if necessary
  for i in range(3 + qoffset):
    robot_output.position[i] = q[7 - qoffset + i]

  # remainder of left leg
  robot_output.position[3 + qoffset] = q[14]  # knee
  robot_output.position[4 + qoffset] = q[20]  # foot
  robot_output.position[5 + qoffset] = q[15]  # shin
  robot_output.position[6 + qoffset] = q[16]  # tarsus
  robot_output.position[7 + qoffset] = q[17]  # heel spring

  # right hip, also in identical order
  robot_output.position[8 + qoffset] = q[21]
  robot_output.position[9 + qoffset] = q[22]
  robot_output.position[10 + qoffset] = q[23]

  # remainder of right leg
  robot_output.position[11 + qoffset] = q[28]  # knee
  robot_output.position[12 + qoffset] = q[34]  # foot
  robot_output.position[13 + qoffset] = q[29]  # shin
  robot_output.position[14 + qoffset] = q[30]  # tarsus
  robot_output.position[15 + qoffset] = q[31]  # heel spring

  # get velocities
  # See cassiemujoco.h cassie_sim_qvel for ordering of v
  # floating base and left hip
  for i in range(3 + voffset):
    robot_output.velocity[i] = v[6 - voffset + i]

  # Convert rotational velocity to global frame
  # rot = R.from_quat([q[6], q[3], q[4], q[5]])
  # robot_output.velocity[3:6] = rot.as_dcm() @ robot_output.velocity[3:6]
  # remainder of left leg
  robot_output.velocity[3 + voffset] = v[12]  # knee
  robot_output.velocity[4 + voffset] = v[18]  # foot
  robot_output.velocity[5 + voffset] = v[13]  # shin
  robot_output.velocity[6 + voffset] = v[14]  # tarsus
  robot_output.velocity[7 + voffset] = v[15]  # heel spring

  # right hip
  robot_output.velocity[8 + voffset] = v[19]
  robot_output.velocity[9 + voffset] = v[20]
  robot_output.velocity[10 + voffset] = v[21]


  # remainder of right leg
  robot_output.velocity[11 + voffset] = v[25]  # knee
  robot_output.velocity[12 + voffset] = v[31]  # foot
  robot_output.velocity[13 + voffset] = v[26]  # shin
  robot_output.velocity[14 + voffset] = v[27]  # tarsus
  robot_output.velocity[15 + voffset] = v[28]  # heel spring

  # Efforts
  # Ordered list of drive_out_t addresses
  for i in range(10):
    robot_output.effort[i] = u[i]

  return robot_output
