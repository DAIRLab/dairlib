import numpy as np
import gym
import time
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv

from drake_cassie_gym import DrakeCassieGym, make_vec_env
from swing_foot_env import SwingFootEnv, make_swing_ft_env_fn, get_default_params
# from radio_swing_foot_env import RadioSwingFootEnv
from reward_osudrl import RewardOSUDRL
from high_level_reward import HighLevelReward
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.cassie.controllers import AlipWalkingControllerFactory
from pydairlib.cassie.simulators import CassieSimDiagram
from pydrake.common.yaml import yaml_load

from pydrake.multibody.plant import MultibodyPlant


def make_env():
    osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'
    '''

    controller_plant = MultibodyPlant(8e-5)
    AddCassieMultibody(controller_plant, None, True, urdf, False, False)
    controller_plant.Finalize()
    controller = AlipWalkingControllerFactory(
        controller_plant, True, osc_gains, osqp_settings)
    '''
    gym_env = DrakeCassieGym(RewardOSUDRL(), 
                             osc_gains,
                             osqp_settings,
                             urdf,
                             visualize=False)
    # gym_env.make(controller)
    return gym_env


def test_vec_env():
    n_envs = 1
    swing_vec_env = make_vec_env(make_swing_ft_env, n_envs)
    actions = [get_default_params() for i in range(n_envs)]
    for i in range(2):
        s, r, d, i = swing_vec_env.step(actions)
        print(r)

    
def test_base_env():
    osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'
    radio = np.zeros(18)

    controller_plant = MultibodyPlant(8e-5)
    AddCassieMultibody(controller_plant, None, True, urdf, False, False)
    controller_plant.Finalize()
    controller = AlipWalkingControllerFactory(
        controller_plant, True, False, osc_gains, osqp_settings)
    gym_env = DrakeCassieGym(reward_func=RewardOSUDRL(), visualize=False)
    gym_env.make(controller)
    state = gym_env.reset()
    
    check_env(gym_env)


def test_swing_foot_env():
    gym_env = SwingFootEnv(reward_func=RewardOSUDRL(), visualize=True)
    for i in range(20):
        s, r, d, i = gym_env.step()
        print(r, d)
        if d:
            gym_env.reset()
            time.sleep(0.1)


def test_radio_swing_ft_env():
    gym_env = RadioSwingFootEnv(reward_func=HighLevelReward(0.5), visualize=True)
    for i in range(20):
        s, r, d, i = gym_env.step()
        # print(r, d)
        if d:
            gym_env.reset()

def main():
    osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'

    radio = np.zeros(18)
    while 1:
        controller_plant = MultibodyPlant(8e-5)
        AddCassieMultibody(controller_plant, None, True, urdf, False, False)
        controller_plant.Finalize()
        controller = AlipWalkingControllerFactory(
            controller_plant, True, False, osc_gains, osqp_settings)
        gym_env = DrakeCassieGym(reward_func=RewardOSUDRL(), visualize=True)
        gym_env.make(controller)
        gym_env.advance_to(5.0)
        gym_env.free_sim()
        gym_env.reset()


if __name__ == '__main__':
    # test_vec_env()
    # main()
    # test_radio_swing_ft_env()
    test_swing_foot_env()
