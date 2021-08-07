from cassiemujoco import *
import time
import numpy as np
from mujoco_lcm_utils import *
# import lcm
from pydrake.lcm import DrakeLcm, Subscriber
import dairlib
import signal


class MujocoCassieSim():

  def __init__(self, sim_dt=5e-5, loss_filename='default_loss_weights'):
    self.realtime_rate = 1.0
    self.cycle_usec = 1000000 / (2000 * self.realtime_rate)
    self.lcm = DrakeLcm()
    self.model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'
    self.robot_output = init_robot_output()
    self.cassie_in = cassie_user_in_t()
    self.cassie_env = CassieSim(model_file)
    self.input_sub = Subscriber(lcm=self.lcm, channel="CASSIE_INPUT", lcm_type=dairlib.lcmt_robot_input)

  def pack_input(cassie_in, u):
    # Set control parameters
    for i in range(10):
      cassie_in.torque[i] = u[i]
    return cassie_in

  def pack_cassie_out():
    cassie_out = cassie_out_t()


  def pack_input_pd():
    u_pd = pd_in_t()
    u_pd.leftLeg.motorPd.torque[3] = 0 # Feedforward torque
    u_pd.leftLeg.motorPd.pTarget[3] = -2
    u_pd.leftLeg.motorPd.pGain[3] = 100
    u_pd.leftLeg.motorPd.dTarget[3] = -2
    u_pd.leftLeg.motorPd.dGain[3] = 10
    u_pd.rightLeg.motorPd = u_pd.leftLeg.motorPd
    return u_pd

  def set_contact_params():
    return

  def convert_drake_state_to_mujoco_state():
    return

  def run_sim(self, x_init):
    self.cassie_env.set_state(x_init)
    u = np.zeros(10)
    for i in range(20000):
      action = self.pack_input(self.cassie_in, u)
      self.cassie_env.step(action)
      t = self.cassie_env.time()
      q = self.cassie_env.qpos()
      v = self.cassie_env.qvel()
      robot_output = self.pack_robot_output(robot_output, q, v, u, t)
      now = time.time()            # get the time
      self.lcm.Publish('CASSIE_STATE_SIMULATION', robot_output.encode())
      elapsed = time.time() - now
      time.sleep(self.cycle_usec * 1e-6 - elapsed)

      # cassie_env.step_pd(u_pd)
      # lcm.HandleSubscriptions(2)
      # u = input_sub.message.efforts
      # import pdb; pdb.set_trace()

      # print(state)

