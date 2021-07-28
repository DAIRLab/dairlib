from cassiemujoco import *
import time
import numpy as np
from mujoco_lcm_utils import *
# import lcm
from pydrake.lcm import DrakeLcm, Subscriber
import dairlib
import signal

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

def main():
  realtime_rate = 2.0
  cycle_usec = 1000000 / (2000 * realtime_rate)
  # timeout_usec = mode == MODE_PD ? 100000 : 10000
  lcm = DrakeLcm()
  model_file = '/home/yangwill/workspace/cassie-mujoco-sim/model/cassie.xml'
  robot_output = init_robot_output()
  cassie_in = cassie_user_in_t()
  cassie_env = CassieSim(model_file)
  input_sub = Subscriber(lcm=lcm, channel="CASSIE_INPUT", lcm_type=dairlib.lcmt_robot_input)
  u = np.zeros(10)
  for i in range(20000):
    action = pack_input(cassie_in, u)
    cassie_env.step(action)
    t = cassie_env.time()
    q = cassie_env.qpos()
    v = cassie_env.qvel()
    robot_output = pack_robot_output(robot_output, q, v, u, t)
    now = time.time()            # get the time
    lcm.Publish('CASSIE_STATE_SIMULATION', robot_output.encode())
    elapsed = time.time() - now
    time.sleep(cycle_usec * 1e-6 - elapsed)

    # cassie_env.step_pd(u_pd)
    # lcm.HandleSubscriptions(2)
    # u = input_sub.message.efforts
    # import pdb; pdb.set_trace()

    # print(state)


if __name__ == '__main__':
  main()
