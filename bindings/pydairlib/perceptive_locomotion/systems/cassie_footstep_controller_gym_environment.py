from dataclasses import dataclass
from os import path
from typing import Dict, Union, Type
from matplotlib.cm import ScalarMappable

import numpy as np
import math
from scipy.spatial.transform import Rotation

from pydairlib.cassie.cassie_utils import AddCassieMultibody

from pydairlib.perceptive_locomotion.diagrams import (
    HikingSimDiagram,
    MpfcOscDiagram,
    MpfcOscDiagramInputType,
    PerceptionModuleDiagram
)

from pydairlib.analysis.mbp_plotting_utils import make_mbp_name_vectors
from pydairlib.systems.footstep_planning import Stance
from pydairlib.systems.plant_visualizer import PlantVisualizer
from pydairlib.systems.perception import GridMapVisualizer
from pydairlib.perceptive_locomotion.systems.alip_lqr_rl import \
    AlipFootstepLQROptions
from pydairlib.perceptive_locomotion.systems.height_map_server \
    import HeightMapServer, HeightMapOptions, HeightMapQueryObject
from pydairlib.perceptive_locomotion.systems.elevation_map_converter \
    import ElevationMappingConverter, ElevationMapOptions, ElevationMapQueryObject
from pydairlib.multibody import SquareSteppingStoneList, ReExpressWorldVector3InBodyYawFrame

from pydrake.systems.analysis import SimulatorStatus
from pydrake.geometry import Rgba
from pydrake.common.cpp_param import List
from pydrake.common.value import Value
from pydrake.multibody.math import SpatialForce

from pydrake.geometry import Meshcat
from pydrake.multibody.plant import (
    MultibodyPlant,
    ExternallyAppliedSpatialForce_,
)
from pydrake.systems.framework import (
    DiscreteValues,
)
from pydrake.common.value import Value, AbstractValue
from pydrake.systems.all import (
    State,
    Diagram,
    EventStatus,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    LeafSystem,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
)

params_folder = "bindings/pydairlib/perceptive_locomotion/params"

class ObservationPublisher(LeafSystem):
    def __init__(self, noise=False, simulate_perception=True):
        LeafSystem.__init__(self)
        self.ns = 4
        self.noise = noise
        self.simulate_perception = simulate_perception
        
        self.init_ = 0
        self.episode_noise = None
        
        if self.simulate_perception:
            self.height = 64
        else:
            self.height = 64

        #self.color_mappable = ScalarMappable(cmap='jet')
        self.input_port_indices = {
            'lqr_reference': self.DeclareVectorInputPort(
                "xd_ud", 6
            ).get_index(),
            # 'x_xd' : self.DeclareVectorInputPort(
            #     "x_xd", self.ns
            # ).get_index(),
            'x' : self.DeclareVectorInputPort(
                "x", self.ns
            ).get_index(),
            'vdes': self.DeclareVectorInputPort(
                'vdes', 2
            ).get_index(),
            'state': self.DeclareVectorInputPort(
                'x_u_t', 59
            ).get_index(),
            'gt_state': self.DeclareVectorInputPort(
                'gt_state', 59
            ).get_index(),
        }
        self.output_port_indices = {
            'observations': self.DeclareVectorOutputPort(
                "observations", 3*self.height*self.height+6+16+23 + 3*self.height*self.height, self.calculate_obs
            ).get_index()
        }
        if self.simulate_perception:
            
            self.input_port_indices.update({'height_map' : self.DeclareAbstractInputPort(
            "elevation_map", model_value=Value(ElevationMapQueryObject())
            ).get_index()})

            self.input_port_indices.update({'height_map_query' : self.DeclareAbstractInputPort(
            "height_map_query", model_value=Value(HeightMapQueryObject())
            ).get_index()})
        
        else:
            self.input_port_indices.update({'height_map' : self.DeclareAbstractInputPort(
            "height_map_query", model_value=Value(HeightMapQueryObject())
            ).get_index()})

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])

    def calculate_obs(self, context: Context, output):
        xd_ud = self.EvalVectorInput(context, self.input_port_indices['lqr_reference'])
        ud = xd_ud.value()[4:]
        
        if self.simulate_perception:
            # Ground truth height map for Critic
            gt_hmap_query = self.EvalAbstractInput(
            context, self.input_port_indices['height_map_query']
            ).get_value()

            gt_hmap = gt_hmap_query.calc_height_map_stance_frame(
                np.array([ud[0], ud[1], 0])
            )

            # Plot ground truth height map
            # gt_hmap_query.plot_surface(
            #     "hmap", gt_hmap_grid_world[0], gt_hmap_grid_world[1],
            #     gt_hmap_grid_world[2], rgba = Rgba(0.95, 0.5, 0.5, 1.0))

        hmap_query = self.EvalAbstractInput(
            context, self.input_port_indices['height_map']
        ).get_value()
        
        if self.noise:
            if self.init_ == 0:
                self.camera_episode_noise = np.random.uniform(low=-0.05, high=0.05)
            # X, Y offset : Shifts the camera 
            camera_step_noise = np.random.uniform(low=-0.02, high=0.02, size=(2,))
            adverserial_offset = self.camera_episode_noise + camera_step_noise
        else:
            adverserial_offset = np.zeros(2,)
        
        hmap = hmap_query.calc_height_map_stance_frame(
            np.array([ud[0], ud[1], 0]), np.append(adverserial_offset, 0)
        )
        
        # Visualize grid map @ stance frame
        hmap_grid_world = hmap_query.calc_height_map_world_frame(
            np.array([ud[0], ud[1], 0]), np.append(adverserial_offset, 0)
        )

        # alip = self.EvalVectorInput(context, self.input_port_indices['x_xd']).get_value()
        alip = self.EvalVectorInput(context, self.input_port_indices['x']).get_value()
        vdes = self.EvalVectorInput(context, self.input_port_indices['vdes']).get_value()
        states = self.EvalVectorInput(context, self.input_port_indices['state']).get_value()
        gt_states = self.EvalVectorInput(context, self.input_port_indices['gt_state']).get_value()
        
        joint_angle = states[7:23] # Only joint angles (reject pelvis)
        joint_angle = [0 if math.isnan(x) else x for x in joint_angle]
        gt_joint_angle = gt_states[:23]

        if self.noise:
            if self.init_ == 0:
                # Offset for hmap per episode
                self.episode_noise = np.random.uniform(low=-0.05, high=0.05)
                self.init_ += 1
            
            # Offset for hmap per step
            height_noise = np.random.uniform(low=-0.02, high=0.02, size=(self.height,self.height))
            step_noise = np.random.uniform(low=-0.02, high=0.02)
            hmap[-1] += height_noise + self.episode_noise + step_noise
            hmap_grid_world[-1] += height_noise + self.episode_noise + step_noise

            alipxy_noise = np.random.uniform(low=-0.05, high=0.05, size=(2,)) 
            aliplxly_noise = np.random.uniform(low=-0.03, high=0.03, size=(2,)) # 20%
            vdes_noise = np.random.uniform(low=-0.03, high=0.03, size=(2,))
            angle_noise = np.random.uniform(low=-0.03, high=0.03, size=(16,))
            alip = alip + np.hstack((alipxy_noise, aliplxly_noise))
            vdes = vdes + vdes_noise
            joint_angle = joint_angle + angle_noise


        # Plot depth map with noise
        hmap_query.plot_surface(
            "hmap", hmap_grid_world[0], hmap_grid_world[1],
            hmap_grid_world[2], rgba = Rgba(0.678, 0.847, 0.902, 1.0))

        hmap = hmap.reshape(-1)
        gt_hmap = gt_hmap.reshape(-1)

        out = np.hstack((hmap, alip, vdes, joint_angle, gt_joint_angle, gt_hmap)) # 24621
        output.set_value(out)

class RewardSystem(LeafSystem):
    def __init__(self, alip_params: AlipFootstepLQROptions, sim_env):
        super().__init__()

        self.params = alip_params
        self.cassie_sim = sim_env
        self.prev_fsm = None

        self.input_port_indices = {
            'lqr_reference': self.DeclareVectorInputPort(
                "xd_ud[x,y]", 6
            ).get_index(),
            'x': self.DeclareVectorInputPort(
                'x', 4
            ).get_index(),
            'fsm': self.DeclareVectorInputPort(
                "fsm", 1
            ).get_index(),
            'time_until_switch': self.DeclareVectorInputPort(
                "time_until_switch", 1
            ).get_index(),
            'footstep_command': self.DeclareVectorInputPort(
                'footstep_command', 3
            ).get_index(),
            'state': self.DeclareVectorInputPort(
                'gt_x_u_t', 59
            ).get_index(),
            'vdes': self.DeclareVectorInputPort(
                'vdes', 2
            ).get_index(),
            'swing_ft_tracking_error': self.DeclareVectorInputPort(
                'swing_ft_tracking_error', 1
            ).get_index(),
        }

        self.output_port_indices = {
            'reward': self.DeclareVectorOutputPort(
                "reward", 1, self.calc_reward
            ).get_index()
        }

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])

    def calc_reward(self, context: Context, output) -> None:

        x = self.EvalVectorInput(context, self.input_port_indices['x']).value()
        footstep_command = self.EvalVectorInput(context, self.input_port_indices['footstep_command']).value()
        u = footstep_command[:2]
        xd_ud = self.EvalVectorInput(context, self.input_port_indices['lqr_reference'])
        xd = xd_ud.value()[:4]
        ud = xd_ud.value()[4:]
        LQRcost = (x - xd).T @ self.params.Q @ (x - xd) + (u - ud).T @ self.params.R @ (u - ud)

        x_u_t = self.EvalVectorInput(context, self.input_port_indices['state']).value()
        pos_vel = x_u_t[:45]
        
        plant = self.cassie_sim.get_plant()
        plant_context = plant.CreateDefaultContext()
        plant.SetPositionsAndVelocities(plant_context, pos_vel)
        
        # Body Frame Velocity
        bf_frame = plant.GetBodyByName("pelvis").body_frame()
        bf_velocity = bf_frame.CalcSpatialVelocityInWorld(plant_context)
        bf_vel = bf_velocity.translational()
        bf_ang = bf_velocity.rotational()
        bf_vel = ReExpressWorldVector3InBodyYawFrame(plant, plant_context, "pelvis", bf_vel)
        bf_ang = ReExpressWorldVector3InBodyYawFrame(plant, plant_context, "pelvis", bf_ang)

        vdes = self.EvalVectorInput(
            context,
            self.input_port_indices['vdes']
        ).value().ravel()

        # bf_matrix = bf_frame.CalcPoseInWorld(plant_context).rotation().matrix()
        # bf_dir = np.dot(bf_matrix[:, 0] ,self.x_axis)
        # rotation = Rotation.from_matrix(bf_matrix)
        # yaw_rad = np.linalg.norm(rotation.as_euler('xyz', degrees=False)[2])

        left_penalty = 0
        right_penalty = 0
        
        # penalize toe angle
        front_contact_pt = np.array((-0.0457, 0.112, 0))
        rear_contact_pt = np.array((0.088, 0, 0))

        left_toe_rotation = plant.GetBodyByName("toe_left").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
        left_toe_direction = left_toe_rotation @ (front_contact_pt - rear_contact_pt)
        left_angle = abs(np.arctan2(left_toe_direction[2], np.linalg.norm(left_toe_direction[:2])))
        
        right_toe_rotation = plant.GetBodyByName("toe_right").body_frame().CalcPoseInWorld(plant_context).rotation().matrix()
        right_toe_direction = right_toe_rotation @ (front_contact_pt - rear_contact_pt)
        right_angle = abs(np.arctan2(right_toe_direction[2], np.linalg.norm(right_toe_direction[:2])))

        track_error = self.EvalVectorInput(context, self.input_port_indices['swing_ft_tracking_error']).value()

        # velocity_reward = np.exp(-3*np.linalg.norm(vdes[:2] - bf_vel[:2]))
        vx_reward = np.exp(-np.linalg.norm(vdes[0] - bf_vel[0]))
        vy_reward = np.exp(-np.linalg.norm(vdes[1] - bf_vel[1]))
        
        # penalize angular velocity about the z axis
        angular_reward = np.exp(-np.linalg.norm(bf_ang))
        track_penalty = 0.0

        if track_error > 0.05:
            track_penalty = min(np.exp(2.5*(track_error-0.05)) - 1, 2)

        fsm = self.EvalVectorInput(context, self.input_port_indices['fsm']).value()

        if left_angle > 0.35:# and (fsm == 0. or fsm == 4.):
            left_penalty = 1.
        if right_angle > 0.35:# and (fsm == 1. or fsm == 3.):
            right_penalty = 1.
        
        if self.prev_fsm == 3. and fsm == 1.:
            ud_penalty = 0.
            LQRreward = 0.
        elif self.prev_fsm == 4. and fsm == 0.:
            ud_penalty = 0.
            LQRreward = 0.
        else:
            LQRreward = np.exp(-3*LQRcost)
            track_penalty = 0.
            ud_penalty = np.linalg.norm(u - ud)

        self.prev_fsm = fsm.copy()

        reward = 0.25 * LQRreward + 0.5 * vx_reward + 0.125 * vy_reward + 0.125 * angular_reward \
                - (left_penalty + right_penalty) - track_penalty - 0.25 * ud_penalty

        output[0] = reward

class DisturbanceSystem(LeafSystem):
    def __init__(self, plant, force_x, force_y, force_z, period, duration):
        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce_[float]]]
        self.DeclareAbstractOutputPort("spatial_forces",
                                        lambda: forces_cls(),
                                        self.calc_dist)
        self.plant = plant
        self.pelvis = self.plant.GetBodyByName("pelvis")
        self.force_x = force_x
        self.force_y = force_y
        self.force_z = force_z
        self.period = period
        self.duration = duration

    def calc_dist(self, context, spatial_forces):
        # Apply force to CoM
        force = ExternallyAppliedSpatialForce_[float]()
        force.body_index = self.pelvis.index()
        force.p_BoBq_B = self.pelvis.default_com()
        y = context.get_time() % self.period
        if not ((y >= 0) and (y <= (self.period - self.duration))):
            rand_force_x = np.random.uniform(low=-self.force_x, high=self.force_x)
            rand_force_y = np.random.uniform(low=-self.force_y, high=self.force_y)
            rand_force_z = np.random.uniform(low=-self.force_z, high=self.force_z)
            spatial_force = SpatialForce(
                tau=[0, 0, 0],
                f=[rand_force_x, rand_force_y, rand_force_z])
        else:
            spatial_force = SpatialForce(
                tau=[0, 0, 0],
                f=[0, 0, 0])
        force.F_Bq_W = spatial_force
        spatial_forces.set_value([force])

class InitialConditionsServer:
    def __init__(self, fname: str):
        datafile = np.load(fname, allow_pickle=True)
        self.data = datafile['arr_0']
        self.idx = 0
        self.rng = np.random.default_rng()

    def next(self):
        if self.idx >= len(self.data):
            return None
        data = self.data[self.idx]
        self.idx += 1
        return data

    def random(self):
        rand = self.rng.choice(self.data)
        return rand

    def choose(self, idx):
        if idx >= len(self.data): # 157869
            return None
        self.idx = idx
        data = self.data[self.idx]
        return data

@dataclass
class CassieFootstepControllerEnvironmentOptions:
    terrain: Union[str, SquareSteppingStoneList] = path.join(
        params_folder, 'terrain.yaml'
    )
    rgdb_extrinsics_yaml: str = path.join(
        params_folder, 'rgbd_extrinsics.yaml'
    )
    osc_gains_yaml: str = path.join(
        params_folder, 'osc_gains.yaml'
    )
    mpfc_gains_yaml: str = path.join(
        params_folder, 'alip_s2s_mpfc_gains.yaml'
    )
    osqp_options_yaml: str = path.join(
        params_folder, 'osqp_options_osc.yaml'
    )
    elevation_mapping_params_yaml: str = path.join(
        params_folder, 'elevation_mapping_params_simulation.yaml'
    )
    urdf: str = "examples/Cassie/urdf/cassie_v2.urdf"
    
    controller_input_type: MpfcOscDiagramInputType = \
        MpfcOscDiagramInputType.kFootstepCommand
    simulate_perception: bool = True
    visualize: bool = False
    meshcat: Meshcat = None

class CassieFootstepControllerEnvironment(Diagram):

    def __init__(self, params: CassieFootstepControllerEnvironmentOptions):
        super().__init__()

        if params.visualize:
            assert params.meshcat is not None
        
        self.params = params
        self.controller_plant = MultibodyPlant(0.0)
        _ = AddCassieMultibody(
            self.controller_plant,
            None,
            True,
            params.urdf,
            True,
            False,
            True
        )
        self.controller_plant.Finalize()
        self.nq = self.controller_plant.num_positions() # 23
        self.nv = self.controller_plant.num_velocities() # 22
        self.na = self.controller_plant.num_actuators() # 10
        self.ns = self.controller_plant.num_multibody_states() # 45
        # q,v,u = make_mbp_name_vectors(self.controller_plant) # For printing out the joints etc.
        # print(v)

        builder = DiagramBuilder()
        self.controller = MpfcOscDiagram(
            self.controller_plant,
            params.osc_gains_yaml,
            params.mpfc_gains_yaml,
            params.osqp_options_yaml,
            params.controller_input_type
        )
        self.cassie_sim = HikingSimDiagram(
            params.terrain,
            params.rgdb_extrinsics_yaml
        )
        self.radio_source = ConstantVectorSource(np.zeros(18, ))

        builder.AddSystem(self.controller)
        builder.AddSystem(self.cassie_sim)
        builder.AddSystem(self.radio_source)

        self.disturbance = True
        if self.disturbance:
            self.disturbance = builder.AddSystem(DisturbanceSystem(plant=self.controller_plant, force_x=30.0, \
            force_y=30.0, force_z=10.0, period=1, duration=0.3))
            builder.Connect(
                    self.disturbance.get_output_port(),
                    self.cassie_sim.get_input_port_spatial_force()
                )

        self.height_map_server = None
        self.perception_module = None
        self.plant_visualizer = PlantVisualizer(params.urdf, params.meshcat) if params.visualize else None

        if params.simulate_perception:
            self.sensor_info = {
                "pelvis_depth":
                    self.cassie_sim.get_depth_camera_info("pelvis_depth"),
            }
            self.perception_module = PerceptionModuleDiagram.Make(
                params.elevation_mapping_params_yaml, self.sensor_info, ""
            )
            builder.AddSystem(self.perception_module)
            builder.Connect(
                self.cassie_sim.get_output_port_cassie_out(),
                self.perception_module.get_input_port_cassie_out()
            )
            #builder.Connect(
            #    self.perception_module.get_output_port_robot_output(),
            #    self.controller.get_input_port_state()
            #)
            builder.Connect(
                self.cassie_sim.get_output_port_depth_image(),
                self.perception_module.get_input_port_depth_image("pelvis_depth")
            )
            elevation_options = ElevationMapOptions()
            elevation_options.meshcat = self.plant_visualizer.get_meshcat() if params.visualize else None
            self.height_map_server = ElevationMappingConverter(
                params.urdf,
                elevation_options
            )
            builder.AddSystem(self.height_map_server)
            builder.Connect(
                self.perception_module.get_output_port_elevation_map(),
                self.height_map_server.get_input_port_by_name('elevation'),
            )
            builder.Connect(
                self.cassie_sim.get_output_port_state_lcm(),
                self.controller.get_input_port_state(),
            )
            builder.Connect(
                self.controller.get_output_port_fsm(),
                self.height_map_server.get_input_port_by_name('fsm')
            )
            builder.Connect(
                self.cassie_sim.get_output_port_state(),
                self.height_map_server.get_input_port_by_name('x')
            )

            # Height_map server for data_collection
            hmap_options = HeightMapOptions()
            hmap_options.meshcat = self.plant_visualizer.get_meshcat() if params.visualize else None
            self.height_map_query_server = HeightMapServer(
                params.terrain,
                params.urdf,
                hmap_options
            )
            builder.AddSystem(self.height_map_query_server)
            builder.Connect(
                self.controller.get_output_port_fsm(),
                self.height_map_query_server.get_input_port_by_name('fsm')
            )
            builder.Connect(
                self.cassie_sim.get_output_port_state(),
                self.height_map_query_server.get_input_port_by_name('x')
            )
        else:
            hmap_options = HeightMapOptions()
            hmap_options.meshcat = self.plant_visualizer.get_meshcat() if params.visualize else None
            self.height_map_server = HeightMapServer(
                params.terrain,
                params.urdf,
                hmap_options
            )
            builder.AddSystem(self.height_map_server)
            builder.Connect(
                self.cassie_sim.get_output_port_state_lcm(),
                self.controller.get_input_port_state(),
            )
            builder.Connect(
                self.controller.get_output_port_fsm(),
                self.height_map_server.get_input_port_by_name('fsm')
            )
            builder.Connect(
                self.cassie_sim.get_output_port_state(),
                self.height_map_server.get_input_port_by_name('x')
            )
        builder.Connect(
            self.cassie_sim.get_output_port_lcm_radio(),
            self.controller.get_input_port_radio()
        )
        builder.Connect(
            self.controller.get_output_port_actuation(),
            self.cassie_sim.get_input_port_actuation()
        )
        builder.Connect(
            self.radio_source.get_output_port(),
            self.cassie_sim.get_input_port_radio()
        )
        if params.visualize:

            builder.AddSystem(self.plant_visualizer)
            self.visualizer = self.cassie_sim.AddDrakeVisualizer(builder)

            # if params.simulate_perception:
                # Visualize depth sensor grid map
                # self.grid_map_visualizer = GridMapVisualizer(
                #    self.plant_visualizer.get_meshcat(), 1.0 / 30.0, ["elevation"]
                # )
                # builder.AddSystem(self.grid_map_visualizer)
            #     builder.Connect(
            #         self.perception_module.get_output_port_state(),
            #         self.plant_visualizer.get_input_port()
            #     )
                # builder.Connect(
                #    self.perception_module.get_output_port_elevation_map(),
                #    self.grid_map_visualizer.get_input_port()
                # )
            # else:
            builder.Connect(
                self.cassie_sim.get_output_port_state(),
                self.plant_visualizer.get_input_port()
            )

        self.input_port_indices = self.export_inputs(builder)
        self.output_port_indices = self.export_outputs(builder)

        builder.BuildInto(self)

    def export_inputs(self, builder: DiagramBuilder) -> Dict[
        str, InputPortIndex]:
        input_port_indices = {
            'footstep_command': builder.ExportInput(
                self.controller.get_input_port_footstep_command(),
                "footstep_command"
            ),
        } if self.params.controller_input_type == MpfcOscDiagramInputType.kFootstepCommand else { ###
            'alip_mpc_output': builder.ExportInput(
                self.controller.get_input_port_alip_mpc_output(),
                "alip_mpc_output"
            ),
        }
        return input_port_indices

    def export_outputs(self, builder: DiagramBuilder) -> Dict[
        str, OutputPortIndex]:
        output_port_indices = {
            'alip_state': builder.ExportOutput(
                self.controller.get_output_port_alip(),
                'alip_state'
            ),
            'fsm': builder.ExportOutput(
                self.controller.get_output_port_fsm(),
                'fsm'
            ),
            'time_until_switch': builder.ExportOutput(
                self.controller.get_output_port_switching_time(),
                'time_until_switch'
            ),
            'swing_ft_tracking_error': builder.ExportOutput(
                self.controller.get_output_port_swing_ft_tracking_error(),
                'swing_ft_tracking_error'
            ),
            'lcmt_cassie_out': builder.ExportOutput(
                self.cassie_sim.get_output_port_cassie_out(),
                'lcmt_cassie_out'
            ),
            'gt_x_u_t': builder.ExportOutput(
                self.cassie_sim.get_output_port_state(),
                'gt_x_u_t'
            )

        }
        if self.params.simulate_perception:
            output_port_indices['height_map'] = builder.ExportOutput(
                self.height_map_server.get_output_port(),
                'elevation_map'
            )
            output_port_indices['height_map_query'] = builder.ExportOutput(
                self.height_map_query_server.get_output_port(),
                'height_map_query'
            )
            output_port_indices['lcmt_robot_output'] = builder.ExportOutput( ###
                self.perception_module.get_output_port_robot_output(),
                'lcmt_robot_output'
            )
            output_port_indices['state'] = builder.ExportOutput( ###
                self.perception_module.get_output_port_state(),
                'x, u, t'
            )
        else:
            output_port_indices['state'] = builder.ExportOutput( ###
                self.cassie_sim.get_output_port_state(),
                'x, u, t'
            )
            output_port_indices['height_map'] = builder.ExportOutput(
                self.height_map_server.get_output_port(),
                'height_map_query'
            )
        return output_port_indices

    def get_input_port_by_name(self, name: str) -> InputPort:
        assert (name in self.input_port_indices)
        return self.get_input_port(self.input_port_indices[name])

    def get_output_port_by_name(self, name: str) -> OutputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])

    def get_heightmap(self, context: Context,
                      center: np.ndarray = None) -> np.ndarray:
        if self.params.simulate_perception:
            raise NotImplementedError(
                "Need to implement heightmap getter for simulated perception."
            )
        else:
            hmap_server_context = self.GetSubsystemContext(
                self.height_map_server, context
            )
            return self.height_map_server.get_height_map_in_stance_frame_from_inputs(
                hmap_server_context, center
            )

    def query_heightmap(self, context: Context,
                        query_point: np.ndarray) -> float:
        robot_output = self.get_output_port_by_name('state').Eval(context)
        fsm = int(self.get_output_port_by_name('fsm').Eval(context)[0])
        stance = Stance.kLeft if fsm == 0 or fsm == 3 else Stance.kRight
        point = query_point.ravel()
        if np.size(point) > 2:
            point = point[:2]

        return self.height_map_server.query_height_in_stance_frame(
            xy=point,
            robot_state=robot_output[:self.nq + self.nv],
            stance=stance
        )

    def initialize_state(self, context: Context, diagram: Diagram,
                   q: np.ndarray = None, v: np.ndarray = None) -> None:
        if q is None:
            q, v = self.cassie_sim.SetPlantInitialConditionFromIK(
                diagram,
                context,
                np.zeros((3,)),
                0.15,
                1.01
            )
        else:
            self.cassie_sim.SetPlantInitialCondition(diagram, context, q, v)
        if self.params.simulate_perception:
            self.perception_module.InitializeEkf(context, q, v)
            self.perception_module.InitializeElevationMap(np.concatenate([q, v]), context)
            
    def AddToBuilderWithFootstepController(
            self, builder: DiagramBuilder,
            footstep_controller_type: Type, **controller_kwargs):
        builder.AddSystem(self)
        footstep_controller = footstep_controller_type(
            self.make_my_controller_options(),
            **controller_kwargs
        )
        builder.AddSystem(footstep_controller)
        builder.Connect(
            self.get_output_port_by_name("fsm"),
            footstep_controller.get_input_port_by_name("fsm")
        )
        builder.Connect(
            self.get_output_port_by_name("time_until_switch"),
            footstep_controller.get_input_port_by_name("time_until_switch")
        )
        builder.Connect(
            self.get_output_port_by_name("alip_state"),
            footstep_controller.get_input_port_by_name("state")
        )
        self.ALIPfootstep_controller = footstep_controller
        return footstep_controller

    def AddToBuilderObservations(self, builder: DiagramBuilder):
        obs_pub = ObservationPublisher(noise=True, simulate_perception=self.params.simulate_perception)
        builder.AddSystem(obs_pub)
        # builder.Connect(
        #     self.ALIPfootstep_controller.get_output_port_by_name("x_xd"), #x_xd
        #     obs_pub.get_input_port_by_name("x_xd")
        # )
        builder.Connect(
            self.ALIPfootstep_controller.get_output_port_by_name("x"),
            obs_pub.get_input_port_by_name("x")
        )
        if self.params.simulate_perception:
            builder.Connect(
                self.get_output_port_by_name("height_map_query"),
                obs_pub.get_input_port_by_name("height_map_query")
            )
        builder.Connect(
            self.get_output_port_by_name("height_map"),
            obs_pub.get_input_port_by_name("height_map")
        )
        builder.Connect(
            self.ALIPfootstep_controller.get_output_port_by_name("lqr_reference"),
            obs_pub.get_input_port_by_name("lqr_reference")
        )
        builder.Connect(
            self.ALIPfootstep_controller.get_output_port_by_name("vdes"),
            obs_pub.get_input_port_by_name("vdes")
        )
        builder.Connect(
            self.get_output_port_by_name("gt_x_u_t"),
            obs_pub.get_input_port_by_name("gt_state")
        )
        builder.Connect(
            self.get_output_port_by_name("state"),
            obs_pub.get_input_port_by_name("state")
        )

        builder.ExportOutput(obs_pub.get_output_port_by_name("observations"), "observations")
        return obs_pub
        

    def AddToBuilderRewards(self, builder: DiagramBuilder):
        footstep_controller = self.ALIPfootstep_controller
        sim_env = self.cassie_sim
        reward = RewardSystem(footstep_controller.params, sim_env)
        builder.AddSystem(reward)

        for controller_port in ['lqr_reference', 'x', 'footstep_command', 'vdes']:
            builder.Connect(
                footstep_controller.get_output_port_by_name(controller_port),
                reward.get_input_port_by_name(controller_port)
            )
        for sim_port in ['fsm', 'time_until_switch']:#, 'state']:
            builder.Connect(
                self.get_output_port_by_name(sim_port),
                reward.get_input_port_by_name(sim_port)
            )

        builder.Connect(
            self.get_output_port_by_name("gt_x_u_t"),
            reward.get_input_port_by_name("state")
        )
        builder.Connect(
            self.get_output_port_by_name("swing_ft_tracking_error"),
            reward.get_input_port_by_name("swing_ft_tracking_error")
        )

        builder.ExportOutput(reward.get_output_port(), "reward")
        return reward

    def make_my_controller_options(self):
        return AlipFootstepLQROptions.calculate_default_options(
            self.params.mpfc_gains_yaml,
            self.controller_plant,
            self.controller_plant.CreateDefaultContext(),
        )


def main():
    opts = CassieFootstepControllerEnvironmentOptions()
    env = CassieFootstepControllerEnvironment(opts)

    builder = DiagramBuilder()
    builder.AddSystem(env)
    footstep_source = ConstantVectorSource(np.array([0.1, -0.3, 0.0]))
    builder.AddSystem(footstep_source)

    builder.Connect(
        footstep_source.get_output_port(),
        env.get_input_port_by_name("footstep_command")
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # repeat the simulation a few times
    for _ in range(5):
        context = diagram.CreateDefaultContext()
        env.cassie_sim.SetPlantInitialConditionFromIK(
            diagram,
            context,
            np.zeros((3,)),
            0.15,
            1.0
        )
        simulator.reset_context(context)
        simulator.Initialize()
        simulator.set_target_realtime_rate(1.0)
        simulator.AdvanceTo(1.0)


if __name__ == '__main__':
    main()
