import numpy as np

from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RigidTransform, RotationMatrix

from pydairlib.multibody.utils import \
    ReExpressWorldVector3InBodyYawFrame,\
    MakeNameToPositionsMap, \
    MakeNameToVelocitiesMap, \
    CreateStateNameVectorFromMap

from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.cassie.controllers import AlipWalkingControllerFactory, \
    FootstepTargetWalkingControllerFactory
from pydairlib.cassie.cassie_gym.drake_cassie_gym import DrakeCassieGym
from pydairlib.cassie.cassie_gym.cassie_env_state import \
    CASSIE_QUATERNION_SLICE, CASSIE_POSITION_SLICE, CASSIE_OMEGA_SLICE,\
    CASSIE_VELOCITY_SLICE, CASSIE_JOINT_POSITION_SLICE,\
    CASSIE_JOINT_VELOCITY_SLICE, CASSIE_FB_POSITION_SLICE,\
    CASSIE_FB_VELOCITY_SLICE


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
        self.pos_map = MakeNameToPositionsMap(self.sim_plant)
        self.vel_map = MakeNameToPositionsMap(self.sim_plant)
        self.state_name_list = CreateStateNameVectorFromMap(self.sim_plant)
        self.swapped_state_names = CreateStateNameVectorFromMap(self.sim_plant)
        for i, name in enumerate(self.swapped_state_names):
            if 'right' in name:
                self.swapped_state_names[i].replace('right', 'left')
            elif 'left' in name:
                self.swapped_state_names[i].replace('left', 'right')
        self.nq = self.sim_plant.num_positions()
        self.nv = self.sim_plant.num_velocities()
        self.calc_context = self.sim_plant.CreateDefatulContext()

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

    def remap_joints_left_to_right(self, x):
        xnew = np.zeros((self.nq + self.nv,))
        for i, name in enumerate(self.swapped_state_names):
            if i < self.nq:
                xnew[self.pos_map[name]] = x[i]
            else:
                xnew[self.vel_map[name]] = x[i]
        return xnew

    '''
        Reflects and remaps the robot state about the pelvis centerline
        assuming we have already transformed the state to a local frame
    '''
    def reflect_state_about_centerline(self, x):
        xnew = np.zeros((self.nq, self.nv))
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, x)
        R_WB = self.pelvis_pose(self.calc_context).rotation().matrix()

        # Construct a matrix R, which reflects 3d vectors
        # across the x-z plane of the pelvis
        n = np.array([[0], [1], [0]])
        R = np.eye(3) - 2 * n @ n.T

        new_R_WB = R @ R_WB
        new_R_WB[:,-1] *= -1
        q = RotationMatrix(new_R_WB).ToQuaternion().wxyz()

        xnew[CASSIE_QUATERNION_SLICE] = q
        xnew[CASSIE_JOINT_POSITION_SLICE] = x[CASSIE_JOINT_POSITION_SLICE]
        xnew[CASSIE_JOINT_VELOCITY_SLICE] = x[CASSIE_JOINT_VELOCITY_SLICE]
        xnew[CASSIE_FB_POSITION_SLICE] = R @ x[CASSIE_FB_POSITION_SLICE]
        xnew[CASSIE_FB_VELOCITY_SLICE] = R @ x[CASSIE_FB_VELOCITY_SLICE]
        xnew[CASSIE_OMEGA_SLICE] = R @ x[CASSIE_OMEGA_SLICE]
        return self.remap_joints_left_to_right(xnew)

    def pelvis_pose(self, context):
        return self.sim_plant.EvalBodyPoseInWorld(
            context=context,
            body=self.sim_plant.GetBodyByName("pelvis"))

    def get_robot_centric_state(self):
        # Get the robot state
        x = self.sim_plant.GetPositionsAndVelocities(self.plant_context)
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, x)

        # Align the robot state with the robot yaw
        b_x = self.pelvis_pose(self.calc_context).col(0).ravel()
        yaw = np.arctan2(b_x[1], b_x[0])
        R = RotationMatrix.MakeZRotation(yaw).inverse().matrix()
        # TODO: apply the transformation and subtract stance foot location

        fsm = self.fsm_output_port.Eval(self.controller_context)
        if self.fsm_state_stance_legs[int(fsm)] == 'right':
            x = self.reflect_state_about_centerline(x)

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

