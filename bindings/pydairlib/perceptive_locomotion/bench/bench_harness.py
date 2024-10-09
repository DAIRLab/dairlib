from dataclasses import dataclass
from os import path
from typing import Dict, Union, Type

import numpy as np

# need to import pydrake first to avoid unknown base class errors
from pydrake.systems.all import (
    State,
    Diagram,
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

from pydairlib.cassie.cassie_utils import AddCassieMultibody

from pydairlib.perceptive_locomotion.diagrams import (
    HikingSimDiagram,
    MpfcOscDiagram,
    MpfcOscDiagramInputType,
    PerceptionModuleDiagram,
    NonRenderPerceptionModuleDiagram,
)

from pydairlib.systems.footstep_planning import Stance
from pydairlib.systems.plant_visualizer import PlantVisualizer
from pydairlib.systems.perception import GridMapVisualizer
from pydairlib.perceptive_locomotion.systems.alip_lqr import \
    AlipFootstepLQROptions
from pydairlib.perceptive_locomotion.systems.height_map_server \
    import HeightMapServer, HeightMapOptions
from pydairlib.perceptive_locomotion.systems.elevation_map_converter \
    import ElevationMappingConverter, ElevationMapOptions

from pydrake.geometry import Meshcat
from pydrake.multibody.plant import MultibodyPlant

from pydairlib.multibody import (
    SquareSteppingStoneList,
    LoadSteppingStonesFromYaml
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

params_folder = "bindings/pydairlib/perceptive_locomotion/params"


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
        return self.rng.choice(self.data)

    def choose(self, idx):
        if idx >= len(self.data):
            return None
        self.idx = idx
        data = self.data[self.idx]
        return data


@dataclass
class BenchEnvOptions:
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

    controller_input_type: MpfcOscDiagramInputType = (
        MpfcOscDiagramInputType.kLcmtAlipMpcOutput)

    simulate_perception: bool = False
    visualize: bool = False
    meshcat: Meshcat = None


class BenchHarness(Diagram):

    def __init__(self, params: BenchEnvOptions):
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
            False
        )
        self.controller_plant.Finalize()
        self.nq = self.controller_plant.num_positions()
        self.nv = self.controller_plant.num_velocities()

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

        self.height_map_server = None
        self.perception_module = None
        self.plant_visualizer = PlantVisualizer(params.urdf, params.meshcat) if params.visualize else None

        self.sensor_info = {
            "pelvis_depth":
                self.cassie_sim.get_depth_camera_info("pelvis_depth"),
        }
        self.perception_module = PerceptionModuleDiagram.Make(
            params.elevation_mapping_params_yaml, self.sensor_info, ""
        )
        builder.AddSystem(self.perception_module)
        builder.Connect(
            self.cassie_sim.get_output_port_depth_image(),
            self.perception_module.get_input_port_depth_image("pelvis_depth")
        )

        builder.Connect(
            self.cassie_sim.get_output_port_cassie_out(),
            self.perception_module.get_input_port_cassie_out()
        )
        builder.Connect(
            self.perception_module.get_output_port_robot_output(),
            self.controller.get_input_port_state()
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
            self.grid_map_visualizer = GridMapVisualizer(
                self.plant_visualizer.get_meshcat(), 1.0 / 30.0, ["elevation"]
            )
            builder.AddSystem(self.grid_map_visualizer)
            builder.Connect(
                self.perception_module.get_output_port_state(),
                self.plant_visualizer.get_input_port()
            )
            builder.Connect(
                self.perception_module.get_output_port_elevation_map(),
                self.grid_map_visualizer.get_input_port()
            )

        self.input_port_indices = self.export_inputs(builder)
        self.output_port_indices = self.export_outputs(builder)

        builder.BuildInto(self)

        DrawAndSaveDiagramGraph(self, '../cassie_footstep_controller.pdf')

    def export_inputs(self, builder: DiagramBuilder) -> Dict[
        str, InputPortIndex]:

        action_port = self.controller.get_input_port_footstep_command() if \
            self.params.controller_input_type == MpfcOscDiagramInputType.kFootstepCommand else \
            self.controller.get_input_port_alip_mpc_output()

        input_port_indices = {
            'command': builder.ExportInput(
                action_port,
                "action"
            ),
        }
        return input_port_indices

    def export_outputs(self, builder: DiagramBuilder) -> Dict[
        str, OutputPortIndex]:
        output_port_indices = {
            'alip_state': builder.ExportOutput(
                self.controller.get_output_port_alip(),
                'alip_state'
            ), 'fsm': builder.ExportOutput(
                self.controller.get_output_port_fsm(),
                'fsm'
            ), 'time_until_switch': builder.ExportOutput(
                self.controller.get_output_port_switching_time(),
                'time_until_switch'
            ), 'swing_ft_tracking_error': builder.ExportOutput(
                self.controller.get_output_port_swing_ft_tracking_error(),
                'swing_ft_tracking_error'
            ), 'pelvis_yaw': builder.ExportOutput(
                self.controller.get_output_port_pelvis_yaw(),
                'pelvis_yaw'
            ), 'lcmt_cassie_out': builder.ExportOutput(
                self.cassie_sim.get_output_port_cassie_out(),
                'lcmt_cassie_out'
            ), 'gt_x_u_t': builder.ExportOutput(
                self.cassie_sim.get_output_port_state(),
                'gt_x_u_t'
            ), 'height_map': builder.ExportOutput(
                self.height_map_server.get_output_port(),
                'height_map (local)'
            ), 'lcmt_robot_output': builder.ExportOutput(
                self.perception_module.get_output_port_robot_output(),
                'lcmt_robot_output'
            ), 'state': builder.ExportOutput(
                self.perception_module.get_output_port_state(),
                'x, u, t'
            ), 'elevation_map': builder.ExportOutput(
                self.perception_module.get_output_port_elevation_map(),
                'elevation_map'
            )
        }
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
            self.perception_module.InitializeEkf(context, q, v)
            self.perception_module.InitializeElevationMap(np.concatenate([q, v]), context)

    def make_my_controller_options(self):
        return AlipFootstepLQROptions.calculate_default_options(
            self.params.mpfc_gains_yaml,
            self.controller_plant,
            self.controller_plant.CreateDefaultContext(),
        )