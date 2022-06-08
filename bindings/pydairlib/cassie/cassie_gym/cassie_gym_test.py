import numpy as np
import gym
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv

from drake_cassie_gym import DrakeCassieGym
from reward_osudrl import RewardOSUDRL
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
    env = SubprocVecEnv([make_env, make_env])
    env.reset()
    s,r,d,i = env.step([np.zeros(18) for i in range(2)])
    print(s)


def test_env():
    osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'
    radio = np.zeros(18)

    controller_plant = MultibodyPlant(8e-5)
    # AddCassieMultibody(controller_plant, None, True, urdf, False, False)
    # controller_plant.Finalize()
    # controller = AlipWalkingControllerFactory(
    #     controller_plant, True, osc_gains, osqp_settings)
    gym_env = DrakeCassieGym(reward_func = RewardOSUDRL(), visualize=False)
    gym_env.make(controller)
    state = gym_env.reset()
    
    check_env(gym_env)

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
            controller_plant, True, osc_gains, osqp_settings)
        gym_env = DrakeCassieGym(reward_func = RewardOSUDRL(), visualize=False)
        gym_env.make(controller)
        gym_env.advance_to(0.35)
        gym_env.free_sim()
        gym_env.reset()


if __name__ == '__main__':
    test_vec_env()
