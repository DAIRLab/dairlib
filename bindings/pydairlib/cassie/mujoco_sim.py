from cassiemujoco import *
import time
import numpy as np
from mujoco_lcm_utils import *
# import lcm
from pydrake.lcm import DrakeLcm, Subscriber
import dairlib
import signal
from pydrake.trajectories import PiecewisePolynomial
from scipy.spatial.transform import Rotation as R
from pydrake.multibody.inverse_kinematics import InverseKinematics
from drake_to_mujoco_converter import DrakeToMujocoConverter


class MujocoCassieSim():

  def __init__(self, sim_dt=5e-5, loss_filename='default_loss_weights', publish_state=True, realtime_rate=1.0):
    self.actuator_index_map = {'hip_roll_left_motor': 0,
                               'hip_yaw_left_motor': 1,
                               'hip_pitch_left_motor': 2,
                               'knee_left_motor': 3,
                               'toe_left_motor': 4,
                               'hip_roll_right_motor': 5,
                               'hip_yaw_right_motor': 6,
                               'hip_pitch_right_motor': 7,
                               'knee_right_motor': 8,
                               'toe_right_motor': 9}
    self.realtime_rate = realtime_rate
    self.cycle_usec = 1000000 / (2000 * self.realtime_rate)
    self.lcm = DrakeLcm()
    self.default_model_directory = '/home/yangwill/workspace/cassie-mujoco-sim/model/'
    self.default_model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'
    self.model_xml = ''
    self.robot_output = init_robot_output()
    self.cassie_out = init_cassie_out()
    self.u_pd = pd_in_t()
    self.cassie_in = cassie_user_in_t()
    self.cassie_env = CassieSim(self.default_model_file)
    # self.cassie_vis = CassieVis(self.cassie_env, self.default_model_file)
    self.input_sub = Subscriber(lcm=self.lcm, channel="CASSIE_INPUT_", lcm_type=dairlib.lcmt_robot_input)
    self.publish_state = publish_state
    self.drake_to_mujoco_converter = DrakeToMujocoConverter(sim_dt)
    self.delay_duration = 0.003 # corresponds to 6 cycles at 2000Hz

  def reinit_env(self, model_file):
    self.cassie_env = CassieSim(model_file)

  def pack_input(self, cassie_in, u_drake):
    act_map = self.drake_to_mujoco_converter.act_map
    # Set control parameters
    u_mujoco = np.zeros(10)
    for u_name in act_map:
      cassie_in.torque[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
      u_mujoco[self.actuator_index_map[u_name]] = u_drake[act_map[u_name]]
    # print('drake:')
    # print(u_drake[5])
    # print(u_mujoco[7])
    return cassie_in, u_mujoco

  def pack_cassie_out(self):
    return self.cassie_out

  def unpack_robot_in(self, robot_in):
    u = np.zeros(10)
    for i in range(10):
      j = self.actuator_index_map[robot_in.effort_names[i]]
      u[j] = robot_in.efforts[i]
    return u

  def pack_input_pd(self):
    self.u_pd.leftLeg.motorPd.torque[3] = 0  # Feedforward torque
    self.u_pd.leftLeg.motorPd.pTarget[3] = -2
    self.u_pd.leftLeg.motorPd.pGain[3] = 100
    self.u_pd.leftLeg.motorPd.dTarget[3] = -2
    self.u_pd.leftLeg.motorPd.dGain[3] = 10
    self.u_pd.rightLeg.motorPd = u_pd.leftLeg.motorPd
    return self.u_pd

  def sim_step(self, u_drake):
    action, u = self.pack_input(self.cassie_in, u_drake)
    self.cassie_env.step(action)
    t = self.cassie_env.time()
    q = self.cassie_env.qpos()
    v = self.cassie_env.qvel()
    # print(v[9:12])
    # print(v[22:25])
    # print(q[10:14])
    # print(q[24:28])
    # print(q)
    # print(v)
    return q, v, u_drake, t

  def set_state(self, x_init):
    q_mujoco, v_mujoco = self.drake_to_mujoco_converter.convert_to_mujoco(x_init)
    # print('omega_init: ')
    # print(v_mujoco[9:12])
    # print(v_mujoco[22:25])
    mujoco_state = self.cassie_env.get_state()
    mujoco_state.set_qpos(q_mujoco)
    mujoco_state.set_qvel(v_mujoco)
    # import pdb; pdb.set_trace()
    # print(v_mujoco)
    self.cassie_env.set_state(mujoco_state)

  # due to the mujoco simulator modeling actuator delays
  # initialize the torque_delay array by running the sim before
  def initialize_motor_efforts(self, start_time, input_traj):
    self.cassie_env.set_time(start_time - self.delay_duration)
    u0 = input_traj.value(start_time)
    while not np.isclose(self.cassie_env.time(), start_time, atol=1e-4):
      _, _, _, _ = self.sim_step(u0)


  def run_sim_playback(self, x_init, start_time, end_time, input_traj=PiecewisePolynomial(np.zeros(10)), params=None):
    # self.model = load_model_from_xml(get_model_xml_text(params))
    # self.cassie_env = CassieSim(load_model_from_xml(get_model_xml_text(params)))
    # print(self.cassie_env.time(), start_time)
    # assert np.isclose(self.cassie_env.time(), start_time)

    self.set_state(x_init)
    self.initialize_motor_efforts(start_time, input_traj)
    self.cassie_env.set_time(start_time)
    self.set_state(x_init)
    # u = np.zeros(10)
    t = start_time
    x_traj = [x_init]
    u_traj = [input_traj.value(t + self.delay_duration)]
    t_traj = [t]
    while self.cassie_env.time() < end_time:
      now = time.time()  # get the time
      u = input_traj.value(t + self.delay_duration)
      # import pdb; pdb.set_trace()
      q, v, u, t = self.sim_step(u)
      # import pdb; pdb.set_trace()
      if (self.publish_state):
        pack_robot_output(self.robot_output, q, v, u, t)
        self.lcm.Publish('CASSIE_STATE_MUJOCO', self.robot_output.encode())
      q, v = self.drake_to_mujoco_converter.convert_to_drake(q, v)
      x_traj.append(np.hstack((q, v)))
      # import pdb; pdb.set_trace()
      u_traj.append(u)
      t_traj.append(t)
      elapsed = time.time() - now
      if(elapsed < self.cycle_usec * 1e-6):
        time.sleep(self.cycle_usec * 1e-6 - elapsed)

    return x_traj, u_traj, t_traj

  def run_sim(self, start_time, end_time, x_init=np.zeros(0)):
    self.cassie_env.set_time(start_time)
    # print(self.cassie_env.qpos())
    # print(self.cassie_env.qvel())
    if x_init.size != 0:
      self.set_state(x_init)
    # self.cassie_vis.draw(self.cassie_env)
    u = np.zeros(10)
    x_traj = []
    u_traj = []
    t_traj = []

    while self.cassie_env.time() < end_time:
      # self.cassie_vis.draw(self.cassie_env)
      now = time.time()  # get the time
      self.lcm.HandleSubscriptions(0)
      if len(self.input_sub.message.efforts) != 0:
        u = self.unpack_robot_in(self.input_sub.message)
      q, v, u, t = self.sim_step(u)
      if (self.publish_state):
        pack_robot_output(self.robot_output, q, v, u, t)
        self.lcm.Publish('CASSIE_STATE_MUJOCO', self.robot_output.encode())
      x_traj.append([q, v])
      u_traj.append(u)
      t_traj.append(t)
      elapsed = time.time() - now
      if(elapsed < self.cycle_usec * 1e-6):
        time.sleep(self.cycle_usec * 1e-6 - elapsed)

    return x_traj, u_traj, t_traj


  # Debugging functions
  def visualize_IK_lower(self, x):
    self.drake_to_mujoco_converter.visualize_state_lower(x)

  def visualize_IK_upper(self, x):
    self.drake_to_mujoco_converter.visualize_state_upper(x)

  def visualize_entire_leg(self, x):
    self.drake_to_mujoco_converter.visualize_entire_leg(x)
