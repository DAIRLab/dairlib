import numpy as np
from dataclasses import dataclass
import csv
import chardet 

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

INITIAL_CONDITIONS_FILE = './learning_data/hardware_ics.npy'
INITIAL_CONDITIONS_FILE_CSV = "./learning_data/cassie_initial_conditions.csv"

@dataclass
class FootstepGymParams(CassieGymParams):
    n_stack: int = 4
    action_rate: int = 10

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


# TODO(hersh500): get this working without depth images
# then add utilities for depth images.
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
        return


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


    # Action space is radio cmd + footstep target for Cassie
    # difficult to find a good stable initial condition to start from
    def step(self, action=None):
        # is sim context the right one to use here?
        pelvis_loc_w = self.pelvis_pose(self.plant_context)
        # radio cmd
        radio = np.zeros((18,))
        if action is None:
            radio[2:6] = 0
            step_location_b = self.alip_target_port.Eval(self.controller_context)
        else:
            radio[2,3,5] = action[:3]
            # footstep target for the current (or upcoming) swing foot
            if self.blind:
                step_location_b = action[3:].append(0) # body frame, for now keeping as 0 (blind)
            else:
                raise NotImplementedError("Perceptive environment is not implemented yet")

        # convert from local body frame to global frame
        step_location_w = step_location_b + self.sim_plant.CalcCenterOfMassPositionInWorld(self.plant_context) 
        # print(f"world frame step target {step_location_w}")

        # get current swing leg 
        swing = self.swing_states[
            int(self.fsm_output_port.Eval(self.controller_context))
        ]
        start_time = self.current_time
        while self.current_time < start_time + 1/self.params.action_rate:
            step_location_b = self.alip_target_port.Eval(self.controller_context)
            step_location_w = step_location_b + self.sim_plant.CalcCenterOfMassPositionInWorld(self.plant_context) 
            s = super().step(fixed_ports=[FixedVectorInputPort(
                        input_port=self.foot_target_input_port,
                        context=self.controller_context,
                        value=step_location_w)])
            s = super().step()
            r = 0  # TODO(hersh500): implement reward for this
            d = super().check_termination()
            if d:
                break
        # TODO(hersh500): select appropriate states (body vel, body rates, body pitch, roll, yaw, foot positions, swing vs. stance states)
        return s, r, d, {}


    # Utilities copied over from StepNetGenerator
    def calc_foot_position_in_world(self, context, side):
        return self.sim_plant.CalcPointsPositions(
            context,
            self.foot_frames[side],
            self.contact_pt_in_ft_frame,
            self.sim_plant.world_frame()).ravel()

    def reset(self, new_x_init=None):
        if new_x_init is not None:
            self.params.x_init = new_x_init
        return super().reset()


    def pelvis_pose(self, context):
        return self.sim_plant.EvalBodyPoseInWorld(
            context=context,
            body=self.sim_plant.GetBodyByName("pelvis"))


    def move_robot_to_origin(self, x):
        self.sim_plant.SetPositionsAndVelocities(self.calc_context, x)
        x[CASSIE_FB_POSITION_SLICE] = (
            x[CASSIE_FB_POSITION_SLICE] -
            self.calc_foot_position_in_world(self.calc_context, 'left')
        )
        return x
   

    # really doesn't like these conditions in the bank, both the hardware and other ones.
    def draw_random_initial_condition_csv(self):
        if self.initial_condition_bank is None:
            reader = csv.reader(open(INITIAL_CONDITIONS_FILE_CSV, 'r'), delimiter=",")
            ics = []
            for row in reader:
                ics.append([float(x) for x in row])
            self.initial_condition_bank = np.array(ics)

        new_ic = csv_ic_to_new_ic(self.initial_condition_bank[:,-1])
        # looks like there might be some chicanery with this.
        # new_ic[13:15] = [-0.0382, -1.823]
        # new_ic[21:23] = [-0.0382, -1.823]
        print(new_ic[7:15])
        print(new_ic[15:23])
        return self.move_robot_to_origin(new_ic.ravel())


    # Returns
    def getBodyHeightFromDepthImage(self, x_b, y_b):
        return 0

    def test_env(self):
        ic = self.draw_random_initial_condition_csv()
        s = self.reset(ic)
        while self.current_time < 5:
            s, r, d, _ = self.step()
            if d:
                break
        return

# test out this env with providing no actions
def main():
    env = CassieFootstepEnv(visualize=True, params=FootstepGymParams())
    env.test_env()

if __name__ == "__main__":
    main()
