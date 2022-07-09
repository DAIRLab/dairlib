import numpy as np

from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from pydrake.trajectories import PiecewisePolynomial

from pydairlib.multibody import \
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
from pydairlib.cassie.cassie_traj_visualizer import CassieTrajVisualizer


class StepnetDataGenerator(DrakeCassieGym):
    '''
    TODO: Add domain randomization for the following:
        Surface Normal
        Initial State
        Footstep target (need to add input port)
    '''
    def __init__(self, visualize=False):
        super().__init__(visualize=visualize)
        # Simulation Infrastructure
        self.fsm_output_port = None
        self.depth_image_output_port = None
        self.foot_target_input_port = None

        # Multibody objects
        self.fsm_state_stance_legs = None
        self.foot_frames = None
        self.front_contact_pt = None
        self.rear_contact_pt = None
        self.mid_contact_pt = None
        self.pos_map = None
        self.vel_map = None
        self.state_name_list = None
        self.state_name_list = None
        self.swapped_state_names = None
        self.nq = 0
        self.nv = 0

        self.calc_context = None

    def make(self, controller, urdf='examples/Cassie/urdf/cassie_v2.urdf'):
        super().make(controller, urdf)
        self.depth_image_output_port = \
            self.cassie_sim.get_camera_out_output_port()
        self.fsm_output_port = \
            self.controller.get_fsm_output_port()
        # self.foot_target_input_port = \
            # self.controller.get_footstep_target_input_port()

        # Multibody objects for Cassie's feet
        self.fsm_state_stance_legs = {0: 'left', 1: 'right',
                                      3: 'left', 4: 'right'}
        self.foot_frames = \
            {'left': self.sim_plant.GetBodyByName("toe_left").body_frame(),
             'right': self.sim_plant.GetBodyByName("toe_right").body_frame()}
        self.front_contact_pt = np.array((-0.0457, 0.112, 0))
        self.rear_contact_pt = np.array((0.088, 0, 0))
        self.mid_contact_pt = 0.5 * (self.front_contact_pt + self.rear_contact_pt)

        # Multibody objects for swapping Cassie's state
        self.pos_map = MakeNameToPositionsMap(self.sim_plant)
        self.vel_map = MakeNameToVelocitiesMap(self.sim_plant)
        self.state_name_list = CreateStateNameVectorFromMap(self.sim_plant)
        self.swapped_state_names = []
        for name in self.state_name_list:
            if 'right' in name:
                self.swapped_state_names.append(name.replace('right', 'left'))
            elif 'left' in name:
                self.swapped_state_names.append(name.replace('left', 'right'))
            else:
                self.swapped_state_names.append(name)
        self.nq = self.sim_plant.num_positions()
        self.nv = self.sim_plant.num_velocities()

        # Spare context for calculations without messing with the sim
        self.calc_context = self.sim_plant.CreateDefaultContext()

    # TODO: modify reset to change initial state, heightmap
    def reset(self):
        super().reset()

    def remap_joints_left_to_right(self, x):
        xnew = np.zeros((self.nq + self.nv,))
        for i, name in enumerate(self.swapped_state_names):
            m = 1
            if 'hip_roll' in name or 'hip_yaw' in name:
                m = -1
            if i < self.nq:
                xnew[self.pos_map[name]] = m * x[i]
            else:
                xnew[self.vel_map[name] + self.nq] = m * x[i]

        return xnew

    '''
        Reflects and remaps the robot state about the pelvis centerline
        assuming we have already transformed the state to a local frame
    '''
    def reflect_state_about_centerline(self, x):
        xnew = np.zeros((self.nq + self.nv,))
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, x)
        R_WB = self.pelvis_pose(self.calc_context).rotation().matrix()

        # Construct a matrix R, which reflects 3d vectors
        # across the x-z plane of the pelvis
        n = np.array([[0], [1], [0]])
        R = np.eye(3) - 2 * n @ n.T

        new_R_WB = R @ R_WB
        new_R_WB[:, 1] *= -1
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

        # Get the current stance leg
        fsm = self.fsm_output_port.Eval(self.controller_context)
        stance = self.fsm_state_stance_legs[int(fsm)]

        # Align the robot state with the robot yaw
        pose = self.pelvis_pose(self.calc_context)
        b_x = pose.rotation().col(0).ravel()
        yaw = np.arctan2(b_x[1], b_x[0])
        R = RotationMatrix.MakeZRotation(yaw).inverse()
        Rmat = R.matrix()
        x[CASSIE_QUATERNION_SLICE] = \
            (R @ RotationMatrix(Quaternion(x[CASSIE_QUATERNION_SLICE]))).ToQuaternion().wxyz()
        x[CASSIE_FB_VELOCITY_SLICE] = Rmat @ x[CASSIE_FB_VELOCITY_SLICE]
        x[CASSIE_OMEGA_SLICE] = Rmat @ x[CASSIE_OMEGA_SLICE]
        x[CASSIE_FB_POSITION_SLICE] = Rmat @ (
                x[CASSIE_FB_POSITION_SLICE] -
                self.sim_plant.CalcPointsPositions(
                    self.calc_context,
                    self.foot_frames[stance],
                    self.mid_contact_pt,
                    self.sim_plant.world_frame()).ravel())

        # Always remap the state to look like it's left stance
        if stance == 'right':
            x = self.reflect_state_about_centerline(x)

        return x

    def get_current_depth_image(self):
        return self.depth_image_output_port.Eval(self.cassie_sim_context)


#TODO: Test state remap with unit test and visual inspection in drake visualizer
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

    t = []
    x = []
    while gym_env.current_time < 10.0:
        gym_env.step()
        t.append(gym_env.current_time)
        x.append(gym_env.get_robot_centric_state()[:23])
        print(gym_env.current_time)
    gym_env.free_sim()

    traj = PiecewisePolynomial.FirstOrderHold(t, np.array(x).T)
    viz = CassieTrajVisualizer(traj)
    while True:
        viz.play()


