import csv
import gym
import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
import time
import yaml
import matplotlib.pyplot as plt

from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.framework import DiagramBuilder, Context, InputPort

from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import DiagramBuilder, Context, InputPort
from pydrake.multibody.plant import *
from pydrake.systems.analysis import Simulator
from pydrake.systems.sensors import ImageToLcmImageArrayT, PixelType
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.lcm import DrakeLcm
from drake import lcmt_image_array

from pydairlib.multibody import \
    ReExpressWorldVector3InBodyYawFrame,\
    MakeNameToPositionsMap, \
    MakeNameToVelocitiesMap, \
    CreateStateNameVectorFromMap
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.cassie.controllers import \
    FootstepTargetWalkingControllerFactory, AlipWalkingControllerFactory
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
from pydairlib.cassie.simulators import CassieSimDiagram, CassieVisionSimDiagram
from pydairlib.cassie.cassie_gym.visual_loco_reward import VisualLocoReward


# Plant and Simulation Constants
OSC_GAINS = 'examples/Cassie/osc/osc_walking_gains_alip.yaml'
OSQP_SETTINGS = 'examples/Cassie/osc/solver_settings/osqp_options_walking.yaml'
URDF = 'examples/Cassie/urdf/cassie_v2_sc.urdf'
MBP_TIMESTEP = 8e-5

CAMERA_CONFIGS = "./systems/cameras/dair_d455.yaml"
INITIAL_CONDITIONS_FILE = './learning_data/initial_conditions.npy'
INITIAL_CONDITIONS_FILE_CSV = "./learning_data/cassie_initial_conditions.csv"

# TODO(hersh500): implement reading this from yaml
@dataclass
class FootstepGymParams(CassieGymParams):
    n_stack: int = 4
    action_rate: int = 5
    reward_weights: tuple = (0.2, -0.2)
    goal_x: float = 4.0
    step_loc_scaling: float = 0.1
    terrain_randomize: bool = False 
    max_initial_x: float = 20 
    min_initial_x: float = -20

    @staticmethod
    def make_random(ic_file_path):
        ics = np.load(ic_file_path)
        x = ics[np.random.choice(ics.shape[0], size=1, replace=False)].ravel()
        normal = np.random.uniform(
            low=[-0.1, -0.1, 1.0],
            high=[0.1, 0.1, 1.0],
        )
        map_yaw = 2 * np.random.random() - 1
        mu = 0.5 * np.random.random() + 0.5
        max_step_magnitude = np.random.uniform(0, 0.1)
        return FootstepGymParams(
            terrain_normal=normal,
            x_init=x,
            map_yaw=map_yaw,
            mu=mu,
            max_step_magnitude = max_step_magnitude
        )

def csv_ic_to_new_ic(csv_ic):
    new_ic = np.zeros(45)
    new_ic[0:7] = csv_ic[0:7]
    new_ic[7:15] = csv_ic[7:22][::2]
    new_ic[15:23] = csv_ic[8:23][::2]
    # Assuming all zero velocities! This is a hack.
    new_ic[23:] = csv_ic[23:]
    return new_ic


class CassieFootstepEnv(DrakeCassieGym):
    # Init should be the same as the StepNet generator stuff
    def __init__(self, visualize=False, blind=True, params=FootstepGymParams()):
        # Initialize the base cassie gym simulator
        super().__init__(
            visualize=visualize,
            params=params
        )

        # should stacking be a feature of the policy or of the environment?
        # for now, let's keep it as a feature of the environment.
        # there's also frame stacking wrapper with stable baslines
        # TODO(hersh500): add framestacking wrapper
        self.n_stack = params.n_stack
        self.blind = blind

        # Simulation Infrastructure
        self.fsm_output_port = None
        self.depth_image_output_port = None
        self.foot_target_input_port = None
        self.alip_target_port = None
        self.initial_condition_bank = None

        # RL definitions
        self.n_states = 14
        self.n_actions = 3 + 2*2
        # TODO(hersh500): these should more tightly fit with the real values.
        self.observation_space = gym.spaces.Box(high=4, low=-4, shape=(self.n_states + self.n_actions,), dtype=np.float32)
        self.action_space = gym.spaces.Box(high=1, low=-1, shape=(self.n_actions,), dtype=np.float32)
        self.reward = VisualLocoReward(self.params.reward_weights)

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

        self.add_controller()
        self.make(self.controller)
        with open(CAMERA_CONFIGS, 'r') as f:
            self.camera_config = yaml.safe_load(f)[self.cassie_sim.get_camera_type()]
        return

  
    def get_camera_matrix(self, config=None):
        if config is None:
            config = self.camera_config
        return np.array([[config["focal_x"], 0, config["center_x"]],
                         [0, config["focal_y"], config["center_y"]],
                         [0, 0, 1]])
            

    def add_controller(self):
        self.controller_plant = MultibodyPlant(MBP_TIMESTEP)
        AddCassieMultibody(self.controller_plant, None, True, URDF, False, False)
        self.controller_plant.Finalize()
        self.controller = FootstepTargetWalkingControllerFactory(
            plant=self.controller_plant,
            has_double_stance=True,
            osc_gains_filename=OSC_GAINS,
            osqp_settings_filename=OSQP_SETTINGS)


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

        self.nq = self.sim_plant.num_positions()
        self.nv = self.sim_plant.num_velocities()
        self.calc_context = self.sim_plant.CreateDefaultContext()


    def check_goal(self, cassie_state):
        return self.cassie_state.get_fb_positions()[0] >= self.params.goal_x

    # Action space is radio cmd + footstep target for Cassie
    # Footstep target is deviation from the alip target.
    def step(self, action=None):
        # Process radio actions
        radio = np.zeros((18,))
        if action is not None:
            # xdot, ydot, yaw_dot
            # Ew.
            radio[2], radio[3], radio[5] = action[0], action[1], action[2]
        else:
            radio[2] = 0.2

        # Process footstep actions
        swing = self.swing_states[
            int(self.fsm_output_port.Eval(self.controller_context))
        ]

        if action is None:
            target = self.alip_target_port.Eval(self.controller_context)
            print(f"target = {target}")
        else:
            # target = np.zeros(3)
            target = self.alip_target_port.Eval(self.controller_context)
            if swing == "right":
                target[0:2] += action[3:5] * self.params.step_loc_scaling
            else:
                target[0:2] += action[5:7] * self.params.step_loc_scaling

        if self.blind:
            alip_target = self.pelvis_pose(self.plant_context).rotation().matrix() @ \
                          target + \
                          self.sim_plant.CalcCenterOfMassPositionInWorld(
                              self.plant_context)
            alip_target[2] = 0.0
        else:
            # TODO(hersh500): lookup the correct height from the depth image.
            image = self.get_current_depth_image()
            # alip_targets are relative to CoM though, and not pelvis. how to reconcile?
            pt_b = self.lookup_height(image, [alip_target[0], alip_target[1]])
        if self.visualize:
            # print(f"radio command = {radio[2:6]}")
            # print(f"step cmd for {swing} foot is {target}")
            pass

        fixed_ports=[FixedVectorInputPort(
            input_port=self.foot_target_input_port,
            context=self.controller_context,
            value=alip_target)]
        start_time = self.current_time
        r = 0
        while self.current_time-start_time < 1/self.params.action_rate:
            s, drake_d = super().step(radio, fixed_ports)
            if self.check_goal(s):
                d = True
                break
            elif drake_d:  # we've fallen over or reached some bad state.
                d = True 
                r = -10
                break
            else:
                d = False
        reduced_state = self.get_reduced_state(s)
        r += self.reward.calc_reward(s)
        return np.append(reduced_state, action), r, d, {}


    def get_reduced_state(self, s):
        rs = np.zeros(self.n_states)
        quat = s.get_orientations()
        euler = R.from_quat(quat).as_euler("xyz")
        rs[3:6] = euler
        rs[0:3] = s.get_fb_velocities()
        # Note this is relative to pelvis not relative to com, could cause issues?
        # TODO(hersh): how to get foot position relative to com?
        rs[6:9] = self.sim_plant.CalcPointsPositions(
                    self.plant_context,
                    self.foot_frames["right"],
                    self.contact_pt_in_ft_frame,
                    self.sim_plant.GetBodyByName("pelvis").body_frame()).flatten()
        rs[9:12] = self.sim_plant.CalcPointsPositions(
                    self.plant_context,
                    self.foot_frames["left"],
                    self.contact_pt_in_ft_frame,
                    self.sim_plant.GetBodyByName("pelvis").body_frame()).flatten()
        swing = int(self.fsm_output_port.Eval(self.controller_context))
        if swing == 0 or swing == 4:
            rs[12] = 1
        else:
            rs[13] = 1
        return rs

    def get_current_depth_image(self):
        return np.copy(
            self.depth_image_output_port.Eval(
                self.diagram.GetMutableSubsystemContext(
                    self.cassie_sim,
                    self.drake_simulator.get_mutable_context()
                )
            ).data
        )


    # gets body-frame z height of the footstep based on the depth image
    # pt_b = (x_b, y_b)
    def lookup_height(self, img, pt_b):
        b2c = self.cassie_sim.get_cam_transform().inverse()
        K = self.get_camera_matrix()
        pt_b1 = np.array([pt_b[0], pt_b[1], -1.1])
        pt_b2 = np.array([pt_b[0], pt_b[1], 0])
        pt_c1 = b2c.multiply(pt_b1)
        pt_c2 = b2c.multiply(pt_b2)
        found_point = False
        theta = 0.01
        while not found_point and theta <= 1:
            # search along upward-pointing ray until pixel z matches cam frame z
            pt_c = pt_c1 + theta * (pt_c2 - pt_c1)
            pt_c_norm = pt_c/pt_c[2]
            pt_im = K @ pt_c_norm
            try:
                im_z = img[int(pt_im[1])][int(pt_im[0])]  # x = row, y = col
            except IndexError:
                return -self.pelvis_pose(self.plant_context).translation()[2]
            if np.abs(im_z - pt_c[2]) < 5e-3:
                found_point = True
            theta += 0.01
        if not found_point:
            return -1.1
        else:
            c2b = b2c.inverse()
            pt_b = c2b.multiply(pt_c)
            return pt_b[2]


    # Utilities copied over from StepNetGenerator
    def calc_foot_position_in_world(self, context, side):
        return self.sim_plant.CalcPointsPositions(
            context,
            self.foot_frames[side],
            self.contact_pt_in_ft_frame,
            self.sim_plant.world_frame()).ravel()


    def reset(self):
        '''
        if new_x_init is not None:
            self.params.x_init = new_x_init
        '''
        # self.params.x_init = self.draw_good_ic() 
        # randomizing initial condition should help with always picking the same action.

        self.params.x_init = self.draw_random_initial_condition()
        s = super().reset()
        actions = np.zeros(self.n_actions)
        return np.append(self.get_reduced_state(s), actions)


    def pelvis_pose(self, context):
        return self.sim_plant.EvalBodyPoseInWorld(
            context=context,
            body=self.sim_plant.GetBodyByName("pelvis"))


    def move_robot(self, x, pos):
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, x)
        x[CASSIE_FB_POSITION_SLICE] = (
            x[CASSIE_FB_POSITION_SLICE] -
            self.calc_foot_position_in_world(self.calc_context, 'left') +
            np.array([pos[0], pos[1], pos[2]])
        )
        # TODO(hersh): should rotate robot to the origin by applying
        # inverse yaw rotation from the rotation matrix. Currently
        # just zero-ing out the yaw but that's hacky.
        rot = x[CASSIE_QUATERNION_SLICE]
        w = rot[0]
        rot[0:3] = rot[1:4]
        rot[3] = w
        euler = R.from_quat(rot).as_euler("zyx")
        euler[0] = 0
        rot = R.from_euler("zyx", euler).as_quat()
        w = rot[3]
        rot[1:4] = rot[0:3]
        rot[0] = w
        x[CASSIE_QUATERNION_SLICE] = rot
        return x
   

    def draw_random_initial_condition(self):
        if self.initial_condition_bank is None:
            self.initial_condition_bank = \
                np.load(INITIAL_CONDITIONS_FILE)
        idx = np.random.choice(
            self.initial_condition_bank.shape[0],
            size=1,
            replace=False)
        if not self.params.terrain_randomize:
            return self.move_robot(
                self.initial_condition_bank[idx].ravel(),
                (0, 0, 0)
            )
        else:
            # pick a position at random
            cond = False
            while not cond:
                new_x = np.random.uniform(self.params.min_initial_x, self.params.max_initial_x)
                height_at_x = self.cassie_sim.get_height_at(new_x, 0)
                plus_x = np.abs(self.cassie_sim.get_height_at(new_x + 0.05, 0) - height_at_x)
                minus_x = np.abs(self.cassie_sim.get_height_at(new_x - 0.05, 0) - height_at_x)
                if plus_x < 1e-2 and minus_x < 1e-2:
                    cond = True
            return self.move_robot(
                self.initial_condition_bank[idx].ravel(),
                (new_x, 0, height_at_x)
            )


    def draw_good_ic(self):
        if self.initial_condition_bank is None:
            self.initial_condition_bank = \
                np.load(INITIAL_CONDITIONS_FILE)
        return self.move_robot_to_origin(self.initial_condition_bank[10].ravel()) 


    def draw_random_initial_condition_csv(self):
        if self.initial_condition_bank is None:
            reader = csv.reader(open(INITIAL_CONDITIONS_FILE_CSV, 'r'), delimiter=",")
            ics = []
            for row in reader:
                ics.append([float(x) for x in row])
            self.initial_condition_bank = np.array(ics)

        new_ic = csv_ic_to_new_ic(self.initial_condition_bank[:,-10])
        # looks like there might be some chicanery with this.
        # new_ic[13:15] = [-0.0382, -1.823]
        # new_ic[21:23] = [-0.0382, -1.823]
        # print(new_ic[7:15])
        # print(new_ic[15:23])
        return self.move_robot_to_origin(new_ic.ravel())


    def test_env(self):
        for i in range(3):
            s = self.reset()
            while self.current_time < 5 and not self.terminated:
                s, r, d, _ = self.step()
        return

    def test_depth_lookup(self):
        s = self.reset()
        s, r, d, i = self.step()
        image = self.get_current_depth_image()
        plt.imshow(image[:,:,0], cmap='gray')
        plt.show()
        pt_b = np.array([0.2, 0])
        z = self.lookup_height(image, pt_b)
        print(f"z = {z}")
        print(f"{self.pelvis_pose(self.plant_context).translation()}")


# is there a good way to reconcile this with the params class?
def make_footstep_env_fn(rank,
                         seed=0,
                         visualize=False,
                         blind=True,
                         goal_x=4.0,
                         max_step_magnitude=0.0,
                         reward_weights=[0.2, -0.1],
                         terrain_randomize=False):
    def _init():
        params = FootstepGymParams(max_step_magnitude=max_step_magnitude, terrain_randomize=terrain_randomize)
        env = CassieFootstepEnv(visualize=visualize, blind=blind, params=params)
        env.seed(seed+rank)
        return env
    return _init


# test out this env with providing no actions
def main():
    env = CassieFootstepEnv(visualize=True, blind=True, params=FootstepGymParams(max_step_magnitude=0.0, terrain_randomize=False))
    env.test_depth_lookup()

if __name__ == "__main__":
    main()
