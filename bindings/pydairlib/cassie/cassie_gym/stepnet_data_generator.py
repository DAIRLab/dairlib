import numpy as np

from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RigidTransform, RotationMatrix

from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.cassie.controllers import AlipWalkingControllerFactory, \
    FootstepTargetWalkingControllerFactory
from pydairlib.cassie.cassie_gym.drake_cassie_gym import DrakeCassieGym


class StepnetDataGenerator(DrakeCassieGym):

    '''
    TODO: Add domain randomization for the following:
        Surface Normal
        Initial State
        Footstep target (need to add input port)
    '''
    def __init__(self, visualize=False):
        super().__init__(visualize=visualize)
        self.fsm_output_port = None
        self.depth_image_output_port = None
        self.foot_target_input_port = None
        self.fsm_state_stance_legs = {0: 'left', 1: 'right',
                                      3: 'left', 4: 'right'}

    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        super().make(controller, urdf)
        self.depth_image_output_port = \
            self.cassie_sim.get_camera_out_output_port()
        self.fsm_output_port = \
            self.controller.get_fsm_output_port()
        self.foot_target_input_port = \
            self.controller.get_footstep_target_input_port()

    # TODO: modify reset to change initial state, heightmap
    def reset(self):
        super().reset()

    def get_robot_centric_state(self):
        # Get the robot state
        x = self.sim_plant.GetPositionsAndVelocities(self.plant_context)
        fsm = self.fsm_output_port.Eval(self.controller_context)
        # TODO: ReExpress quantities in body yaw frame, relabel joints based on fsm

        return x

    def get_current_depth_image(self):
        return self.depth_image_output_port.Eval(self.cassie_sim_context)


def test_data_collection():
    # Cassie settings
    osc_gains = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
    osqp_settings = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
    urdf = 'examples/Cassie/urdf/cassie_v2.urdf'

    # Make the controller
    gym_env = StepnetDataGenerator()
    controller_plant = MultibodyPlant(8e-5)
    AddCassieMultibody(controller_plant, None, True, urdf, False, False)
    controller_plant.Finalize()
    controller = AlipWalkingControllerFactory(
        controller_plant, True, osc_gains, osqp_settings)
    gym_env.make(controller)
    while gym_env.current_time < 10.0:
        gym_env.step()
        print(gym_env.get_robot_centric_state()[0])
    gym_env.free_sim()
    gym_env.reset()

