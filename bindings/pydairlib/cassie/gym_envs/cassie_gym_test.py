from drake_cassie_gym import *
from mujoco_cassie_gym import *
# from cassie_utils import *
from pydairlib.cassie.controllers import OSCRunningControllerFactory
from pydairlib.cassie.controllers import OSCWalkingControllerFactory
from pydairlib.cassie.simulators import CassieSimDiagram
from pydairlib.cassie.gym_envs.reward_osudrl import RewardOSUDRL
from pydrake.common.yaml import yaml_load


def main():
    osc_running_gains_filename = 'examples/Cassie/osc_run/osc_running_gains.yaml'
    # osc_running_gains_filename = 'examples/Cassie/osc_run/learned_osc_running_gains.yaml'
    osc_walking_gains_filename = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc_run/osc_running_qp_settings.yaml'
    default_osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2_conservative.urdf'
    reward_function_weights = ''

    action = np.zeros(18)
    cumulative_reward = 0
    # while 1:
    controller_plant = MultibodyPlant(1e-3)
    AddCassieMultibody(controller_plant, None, True, urdf, False, False, False)
    controller_plant.Finalize()
    controller = OSCRunningControllerFactory(controller_plant, osc_running_gains_filename, osqp_settings)
    # controller = OSCWalkingControllerFactory(controller_plant, True, osc_walking_gains_filename, osqp_settings)

    # reward_func = RewardOSUDRL(reward_function_weights)
    reward_func = RewardOSUDRL()

    gym_env = DrakeCassieGym(reward_func, visualize=True)
    # gym_env = MuJoCoCassieGym(reward_func, visualize=True)
    gym_env.make(controller)
    while(1):
        cumulative_reward = gym_env.advance_to(25)
        gym_env.reset()
    # print(cumulative_reward)
    gym_env.free_sim()
    # while gym_env.current_time < 5.0:
    #     state, reward = gym_env.step(action)
    #     cumulative_reward += reward
    # print(cumulative_reward)

if __name__ == '__main__':
    main()
