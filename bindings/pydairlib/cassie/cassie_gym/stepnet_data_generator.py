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
from pydairlib.cassie.controllers import \
    FootstepTargetWalkingControllerFactory
from pydairlib.cassie.cassie_gym.drake_cassie_gym import \
    DrakeCassieGym,\
    CassieGymParams,\
    FixedVectorInputPort
from pydairlib.cassie.cassie_gym.cassie_env_state import \
    CASSIE_QUATERNION_SLICE, CASSIE_POSITION_SLICE, CASSIE_OMEGA_SLICE,\
    CASSIE_VELOCITY_SLICE, CASSIE_JOINT_POSITION_SLICE,\
    CASSIE_JOINT_VELOCITY_SLICE, CASSIE_FB_POSITION_SLICE,\
    CASSIE_FB_VELOCITY_SLICE
from pydairlib.cassie.cassie_traj_visualizer import CassieTrajVisualizer

# Plant and Simulation Constants
OSC_GAINS = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
OSQP_SETTINGS = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
URDF = 'examples/Cassie/urdf/cassie_v2.urdf'
MBP_TIMESTEP = 8e-5

# Data Collection Constants
INITIAL_CONDITIONS_FILE = '.learning_data/hardware_ics.npy'
TARGET_BOUND = np.array([0.1, 0.1, 0.0])
RADIO_BOUND = np.array([1.0, 1.0])
MAX_ERROR = 1.0


class StepnetDataGenerator(DrakeCassieGym):

    def __init__(self, visualize=False, params=CassieGymParams()):
        # Initialize the base cassie gym simulator
        super().__init__(
            visualize=visualize,
            params=params
        )
        # simulate for a bit past a full step
        # self.sim_dt = 0.45

        # Simulation Infrastructure
        self.fsm_output_port = None
        self.depth_image_output_port = None
        self.foot_target_input_port = None
        self.alip_target_port = None
        self.initial_condition_bank = None

        # Multibody objects
        self.fsm_state_stances = {
            0: 'left',
            1: 'right',
            3: 'right',
            4: 'left'
        }
        self.swing_states = {
            0: 'right',
            1: 'left',
            3: 'left',
            4: 'right'
        }
        self.foot_frames = None
        self.contact_pt_in_ft_frame = None
        self.pos_map = None
        self.vel_map = None
        self.state_name_list = None
        self.state_name_list = None
        self.swapped_state_names = None
        self.nq = 0
        self.nv = 0

        # Spare plant context for calculations
        self.calc_context = None

        self.add_controller()
        self.make(self.controller)

    def add_controller(self):
        self.controller_plant = MultibodyPlant(MBP_TIMESTEP)
        AddCassieMultibody(self.controller_plant, None, True, URDF, False, False)
        self.controller_plant.Finalize()
        self.controller = FootstepTargetWalkingControllerFactory(
            plant=self.controller_plant,
            has_double_stance=True,
            osc_gains_filename=OSC_GAINS,
            osqp_settings_filename=OSQP_SETTINGS
        )

    def make(self, controller, urdf=URDF):
        super().make(controller, urdf)
        self.depth_image_output_port = \
            self.cassie_sim.get_camera_out_output_port()
        self.fsm_output_port = \
            self.controller.get_fsm_output_port()
        self.foot_target_input_port = \
            self.controller.get_footstep_target_input_port()
        self.alip_target_port = \
            self.controller.get_alip_target_footstep_port()

        # Multibody objects for Cassie's feet
        self.foot_frames = {
            'left': self.sim_plant.GetBodyByName("toe_left").body_frame(),
             'right': self.sim_plant.GetBodyByName("toe_right").body_frame()
        }
        self.contact_pt_in_ft_frame = np.array([0.02115, 0.056, 0])

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

    def reset(self, new_x_init=None):
        if new_x_init is not None:
            self.params.x_init = new_x_init
        super().reset()

    def remap_joints_left_to_right(self, x):
        xnew = np.zeros((self.nq + self.nv,))
        for i, name in enumerate(self.swapped_state_names):
            # invert the position and velocity of the hip roll/yaw joints
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

        # Reflect the pelvis orientation across the world xz plane then reflect
        # it again across the pelvis xz plane to maintain a right-handed
        # coordinate system
        new_R_WB = R @ R_WB
        new_R_WB[:, 1] *= -1
        q = RotationMatrix(new_R_WB).ToQuaternion().wxyz()

        # Assign the transformed state
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

    def calc_foot_position_in_world(self, context, side):
        return self.sim_plant.CalcPointsPositions(
            context,
            self.foot_frames[side],
            self.contact_pt_in_ft_frame,
            self.sim_plant.world_frame()).ravel()

    def make_world_to_robot_yaw_rotation(self, context):
        pose = self.pelvis_pose(context)
        b_x = pose.rotation().col(0).ravel()
        yaw = np.arctan2(b_x[1], b_x[0])
        return RotationMatrix.MakeZRotation(yaw).inverse()

    def get_robot_centric_state(self, x):
        # Get the robot state
        xc = np.copy(x)
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, xc)

        # Get the current stance leg
        fsm = self.fsm_output_port.Eval(self.controller_context)
        stance = self.fsm_state_stances[int(fsm)]

        # Align the robot state with the robot yaw
        R = self.make_world_to_robot_yaw_rotation(self.calc_context)
        Rmat = R.matrix()
        xc[CASSIE_QUATERNION_SLICE] = \
            (R @ self.pelvis_pose(self.calc_context).rotation()).ToQuaternion().wxyz()
        xc[CASSIE_FB_VELOCITY_SLICE] = Rmat @ xc[CASSIE_FB_VELOCITY_SLICE]
        xc[CASSIE_OMEGA_SLICE] = Rmat @ xc[CASSIE_OMEGA_SLICE]
        xc[CASSIE_FB_POSITION_SLICE] = Rmat @ (
            xc[CASSIE_FB_POSITION_SLICE] -
            self.calc_foot_position_in_world(self.calc_context, stance)
        )

        # Always remap the state to look like it's left stance
        if stance == 'right':
            xc = self.reflect_state_about_centerline(xc)
        return xc

    def move_robot_to_origin(self, x):
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, x)
        x[CASSIE_FB_POSITION_SLICE] = (
            x[CASSIE_FB_POSITION_SLICE] -
            self.calc_foot_position_in_world(self.calc_context, 'left')
        )
        return x

    def get_current_depth_image(self):
        return np.copy(
            self.depth_image_output_port.Eval(
                self.diagram.GetMutableSubsystemContext(
                    self.cassie_sim,
                    self.drake_simulator.get_mutable_context()
                )
            ).data
        )

    def step(self, footstep_target):
        return super().step(fixed_ports=[FixedVectorInputPort(
            input_port=self.foot_target_input_port,
            context=self.controller_context,
            value=footstep_target
        )])

    def draw_random_initial_condition(self):
        if self.initial_condition_bank is None:
            self.initial_condition_bank = \
                np.load(INITIAL_CONDITIONS_FILE)
        return self.move_robot_to_origin(
            self.initial_condition_bank[
                np.random.choice(
                    self.initial_condition_bank.shape[0],
                    size=1,
                    replace=False
                )
            ].ravel()
        )

    def get_stepnet_data_point(self, seed=0):
        np.random.seed(seed)

        # Reset the simulation to a random initial state
        x_pre = self.draw_random_initial_condition()
        self.reset(x_pre)
        x = self.get_robot_centric_state(x_pre)

        # Apply a random velocity command
        radio = np.zeros((18,))
        radio[2:4] = np.random.uniform(
            low=-RADIO_BOUND,
            high=RADIO_BOUND
        )
        self.cassie_sim.get_radio_input_port().FixValue(
            context=self.cassie_sim_context,
            value=radio)

        # Get the depth image and the current ALIP footstep target
        depth_image = self.get_current_depth_image()
        alip_target = self.pelvis_pose(self.plant_context).rotation.matrix() @ \
                      self.alip_target_port.Eval(self.controller_context) + \
                      self.sim_plant.CalcCenterOfMassPositionInWorld(
                          self.plant_context
                      )
        # Apply a random offset to the target XY position
        target = alip_target + np.random.uniform(
            low=-TARGET_BOUND,
            high=TARGET_BOUND
        )
        target[2] = 0.0

        # Get the current swing leg
        swing = self.swing_states[
            int(self.fsm_output_port.Eval(self.controller_context))
        ]


        # Step the simulation forward until middle of next double stance
        while self.current_time < 0.35:
            self.step(footstep_target=target)
            # Abort and return max error if something crazy happens
            if self.check_termination():
                return {
                    'depth': depth_image,
                    'state': x,
                    'target': target,
                    'error': MAX_ERROR
                }

        # Calculate error
        swing_foot_final_pos = self.calc_foot_position_in_world(
            self.plant_context, swing)
        error = min(MAX_ERROR, np.linalg.norm(swing_foot_final_pos - target))

        return {
            'depth': depth_image,
            'state': x,
            'target': target,
            'error': error
        }

    @staticmethod
    def make_randomized_env(visualize=False):
        env = StepnetDataGenerator(
            visualize=visualize,
            params=CassieGymParams.make_random(INITIAL_CONDITIONS_FILE)
        )
        env.reset(env.get_robot_centric_state(env.params.x_init))
        return env


def test_data_collection():
    gym_env = StepnetDataGenerator.make_randomized_env(visualize=True)
    while True:
        data = gym_env.get_stepnet_data_point()
    gym_env.free_sim()

    # traj = PiecewisePolynomial.FirstOrderHold(t, np.array(x).T)
    # viz = CassieTrajVisualizer(traj)
    # while True:
    #     viz.play()


