import time
import numpy as np
from pydantic import BaseModel, Field

from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.sensors import CameraInfo
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
from pydairlib.cassie.cassie_gym.config_types import \
    CassieGymParams,\
    DepthCameraInfo,\
    DataGeneratorParams,\
    DomainRandomizationBounds
from pydairlib.cassie.cassie_gym.drake_cassie_gym import \
    DrakeCassieGym,\
    FixedVectorInputPort
from pydairlib.cassie.simulators import CassieVisionSimDiagram
from pydairlib.cassie.cassie_gym.cassie_env_state import \
    CASSIE_QUATERNION_SLICE, CASSIE_POSITION_SLICE, CASSIE_OMEGA_SLICE,\
    CASSIE_VELOCITY_SLICE, CASSIE_JOINT_POSITION_SLICE,\
    CASSIE_JOINT_VELOCITY_SLICE, CASSIE_FB_POSITION_SLICE,\
    CASSIE_FB_VELOCITY_SLICE
from pydairlib.cassie.cassie_traj_visualizer import CassieTrajVisualizer

# Plant and Simulation Constants
OSC_GAINS = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
OSQP_SETTINGS = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
SIM_URDF = 'examples/Cassie/urdf/cassie_v2_self_collision.urdf'
URDF = 'examples/Cassie/urdf/cassie_v2.urdf'
MBP_TIMESTEP = 8e-5

# Data Collection Constants
INITIAL_CONDITIONS_FILE = '.learning_data/hardware_ics.npy'
MAX_ERROR = 1.0


class StepnetDataGenerator(DrakeCassieGym):

    def __init__(self, data_gen_params,
                 visualize=False, gym_params=CassieGymParams()):
        # Initialize the base cassie gym simulator
        super().__init__(
            visualize=visualize,
            params=gym_params
        )
        self.data_gen_params = data_gen_params
        self.simulate_until = 0
        self.ss_time = 0

        # Simulation Infrastructure
        self.fsm_output_port = None
        self.depth_image_output_port = None
        self.depth_camera_pose_output_port = None
        self.foot_target_input_port = None
        self.alip_target_port = None
        self.initial_condition_bank = None
        self.com_z_port = None

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

        # Depth camera
        self.depth_camera_info = \
            self.data_gen_params.depth_camera_info.as_drake_camera_info()

        # Spare plant context for calculations
        self.calc_context = None

        self.add_controller()
        self.make(self.controller)

    def add_controller(self):
        self.controller_plant = MultibodyPlant(MBP_TIMESTEP)
        AddCassieMultibody(self.controller_plant, None, True, URDF, False, False)
        self.controller_plant.Finalize()
        if self.data_gen_params.randomize_time:
            self.ss_time = np.random.uniform(
                low=self.data_gen_params.target_time_bounds[0],
                high=self.data_gen_params.target_time_bounds[1]
            )
            self.controller = FootstepTargetWalkingControllerFactory(
                plant=self.controller_plant,
                has_double_stance=True,
                osc_gains_filename=OSC_GAINS,
                osqp_settings_filename=OSQP_SETTINGS,
                single_stance_time_override=self.ss_time
            )
        else:
            self.ss_time = 0.3
            self.controller = FootstepTargetWalkingControllerFactory(
                plant=self.controller_plant,
                has_double_stance=True,
                osc_gains_filename=OSC_GAINS,
                osqp_settings_filename=OSQP_SETTINGS
            )

        self.simulate_until = self.ss_time + 0.25

    def make(self, controller, urdf=SIM_URDF):
        super().make(controller, urdf=urdf)
        self.depth_image_output_port = \
            self.cassie_sim.get_camera_out_output_port()
        self.depth_camera_pose_output_port = \
            self.cassie_sim.get_camera_pose_output_port()
        self.fsm_output_port = \
            self.controller.get_fsm_output_port()
        self.foot_target_input_port = \
            self.controller.get_footstep_target_input_port()
        self.alip_target_port = \
            self.controller.get_alip_target_footstep_port()
        self.com_z_port = \
            self.controller.get_com_z_input_port()

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

    def make_robot_yaw_to_world_rotation(self, context):
        pose = self.pelvis_pose(context)
        b_x = pose.rotation().col(0).ravel()
        yaw = np.arctan2(b_x[1], b_x[0])
        return RotationMatrix.MakeZRotation(yaw)

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

    def get_noisy_height_from_current_depth_image(self, var_z, target_w):
        image = self.depth_image_output_port.Eval(
            self.diagram.GetMutableSubsystemContext(
                self.cassie_sim,
                self.drake_simulator.get_mutable_context()
            )
        ).data
        # Get the depth camera pose in the world frame
        X_WC = self.depth_camera_pose_output_port.Eval(
            self.diagram.GetMutableSubsystemContext(
                self.cassie_sim,
                self.drake_simulator.get_mutable_context()
            )
        )

        X_CW = X_WC.inverse()
        K = self.depth_camera_info.intrinsic_matrix()
        pt_w1 = np.array([target_w[0], target_w[1], -0.5])
        pt_w2 = np.array([target_w[0], target_w[1], 1.0])
        pt_c1 = X_CW.multiply(pt_w1)
        pt_c2 = X_CW.multiply(pt_w2)

        found_point = False
        theta = 0.01
        while not found_point and theta <= 1:
            # search along upward-pointing ray until pixel z matches cam frame z
            pt_c = pt_c1 + theta * (pt_c2 - pt_c1)
            pt_c_norm = pt_c/pt_c[2]
            pt_im = (K @ pt_c_norm).astype('int')
            try:
                im_z = image[pt_im[1]][pt_im[0]]  # x = row, y = col
            except IndexError:
                theta += .005
                continue
            if np.abs(im_z - pt_c[2]) < 5e-3:
                found_point = True
            theta += 0.01
        if not found_point:
            return np.copy(image), np.random.normal(scale=var_z)
        else:
            pt_b = X_WC.multiply(pt_c)
            return np.copy(image), pt_b[2] + np.random.normal(scale=var_z)

    def step(self, footstep_target: np.ndarray, radio: np.ndarray):
        return super().step(
            radio=radio,
            fixed_ports=[
                FixedVectorInputPort(
                    input_port=self.foot_target_input_port,
                    context=self.controller_context,
                    value=footstep_target
                ),
                FixedVectorInputPort(
                    input_port=self.com_z_port,
                    context=self.controller_context,
                    value=np.array([0.85+footstep_target[2]])
                )
            ]
        )

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

    def get_footstep_target_with_random_offset(self):
        # Apply a random velocity command
        radio = np.zeros((18,))
        radio[2:4] = np.random.uniform(
            low=self.data_gen_params.radio_lb,
            high=self.data_gen_params.radio_ub
        )
        if self.data_gen_params.randomize_yaw:
            radio[9] = np.random.uniform(
                low=-self.data_gen_params.target_yaw_noise_bound,
                high=self.data_gen_params.target_yaw_noise_bound
            )

        self.cassie_sim.get_radio_input_port().FixValue(
            context=self.cassie_sim_context,
            value=radio
        )
        # Get the current ALIP footstep target in the body yaw frame
        com_w = self.sim_plant.CalcCenterOfMassPositionInWorld(
            self.plant_context)
        alip_target_b = self.alip_target_port.Eval(self.controller_context) + \
            ReExpressWorldVector3InBodyYawFrame(
                self.plant, self.plant_context, "pelvis", com_w)

        # Apply a random offset to the target XY position
        target = alip_target_b + np.random.uniform(
            low=-self.data_gen_params.target_xyz_noise_bound,
            high=self.data_gen_params.target_xyz_noise_bound
        )
        target_b = np.clip(
            target,
            self.data_gen_params.target_lb,
            self.data_gen_params.target_ub
        )
        target_w = self.make_robot_yaw_to_world_rotation(
            self.plant_context).matrix() @ target_b

        return target_w, target_b, radio

    def get_stepnet_data_point(self, seed=0):
        np.random.seed(seed)

        # Reset the simulation to a random initial state
        x_pre = self.draw_random_initial_condition()
        self.reset(x_pre)
        x = self.get_robot_centric_state(x_pre)

        target_w, target_b, radio = \
            self.get_footstep_target_with_random_offset()

        depth_image, target_z = \
            self.get_noisy_height_from_current_depth_image(
                self.data_gen_params.depth_var_z, target_w
            )
        target_w[-1] = target_z

        # Get the current swing leg
        swing = self.swing_states[
            int(self.fsm_output_port.Eval(self.controller_context))
        ]

        # Step the simulation forward until middle of next double stance
        while self.current_time < self.simulate_until:
            self.step(footstep_target=target_w, radio=radio)
            # Abort and return max error if something crazy happens
            if self.check_termination():
                return {
                    'depth': depth_image,
                    'state': x,
                    'target': {'xyz': target_b, 'yaw': radio[9]},
                    'time': self.ss_time,
                    'error': MAX_ERROR
                }

        # Calculate error
        swing_foot_final_pos = self.calc_foot_position_in_world(
            self.plant_context, swing)
        error = min(MAX_ERROR, np.linalg.norm(swing_foot_final_pos - target_w))

        return {
            'depth': depth_image,
            'state': x,
            'target': {'xyz': target_b, 'yaw': radio[9]},
            'time': self.ss_time,
            'error': error
        }

    def get_flat_ground_stepnet_datapoint(self, seed=0):
        np.random.seed(seed)

        # Reset the simulation to a random initial state
        x_pre = self.draw_random_initial_condition()
        self.reset(x_pre)
        x = self.get_robot_centric_state(x_pre)

        target_w, target_b, radio = \
            self.get_footstep_target_with_random_offset()
        target_w[-1] = 0
        target_b[-1] = 0

        # Get the current swing leg
        swing = self.swing_states[
            int(self.fsm_output_port.Eval(self.controller_context))
        ]

        # Step the simulation forward until about the
        # middle of next double stance
        while self.current_time < self.simulate_until:
            self.step(footstep_target=target_w, radio=radio)
            # Abort and return max error if something crazy happens
            if self.check_termination():
                return {
                    'state': x,
                    'target': {'xyz': target_b, 'yaw': radio[9]},
                    'time': self.ss_time,
                    'error': MAX_ERROR
                }

        # Calculate error
        swing_foot_final_pos = self.calc_foot_position_in_world(
            self.plant_context, swing)
        error = min(MAX_ERROR, np.linalg.norm(swing_foot_final_pos - target_w))

        return {
            'state': x,
            'target': {'xyz': target_b, 'yaw': radio[9]},
            'time': self.ss_time,
            'error': error
        }

    @staticmethod
    def make_randomized_env(data_gen_params, rand_domain, visualize=False):
        env = StepnetDataGenerator(
            data_gen_params,
            visualize=visualize,
            gym_params=CassieGymParams.make_random(
                INITIAL_CONDITIONS_FILE,
                domain=rand_domain
            )
        )
        env.reset(env.get_robot_centric_state(env.params.x_init))
        return env

    @staticmethod
    def make_flat_env(data_gen_params, visualize=False):
        env = StepnetDataGenerator(
            data_gen_params,
            visualize=visualize,
            gym_params=CassieGymParams.make_flat(INITIAL_CONDITIONS_FILE)
        )
        env.reset(env.get_robot_centric_state(env.params.x_init))
        return env


def test_data_collection():
    gym_env = StepnetDataGenerator.make_randomized_env(
        DataGeneratorParams(), DomainRandomizationBounds(), visualize=True)
    i = 0
    while True:
        data = gym_env.get_stepnet_data_point(seed=i)
        i += 1
    gym_env.free_sim()


def test_flat_collection():
    gym_env = StepnetDataGenerator.make_flat_env(
        DataGeneratorParams(), visualize=True)
    i = 0
    while True:
        data = gym_env.get_flat_ground_stepnet_datapoint(seed = i)
        i += 1
    gym_env.free_sim()

    # traj = PiecewisePolynomial.FirstOrderHold(t, np.array(x).T)
    # viz = CassieTrajVisualizer(traj)
    # while True:
    #     viz.play()


