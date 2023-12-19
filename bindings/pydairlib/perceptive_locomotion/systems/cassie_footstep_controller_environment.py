from dataclasses import dataclass
from os import path
from typing import Dict, Union, Type

import numpy as np

from pydairlib.cassie.cassie_utils import AddCassieMultibody

from pydairlib.perceptive_locomotion.diagrams import (
    HikingSimDiagram,
    MpfcOscDiagram,
    PerceptionModuleDiagram
)

from pydairlib.systems.footstep_planning import Stance
from pydairlib.systems.plant_visualizer import PlantVisualizer
from pydairlib.systems.perception import GridMapVisualizer
from pydairlib.perceptive_locomotion.systems.alip_lqr import \
    AlipFootstepLQROptions
from pydairlib.perceptive_locomotion.systems.height_map_server \
    import HeightMapServer, HeightMapOptions
from pydairlib.multibody import SquareSteppingStoneList

from pydrake.multibody.plant import MultibodyPlant
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
        params_folder, 'mpfc_gains.yaml'
    )
    osqp_options_yaml: str = path.join(
        params_folder, 'osqp_options_osc.yaml'
    )
    elevation_mapping_params_yaml: str = path.join(
        params_folder, 'elevation_mapping_params.yaml'
    )
    urdf: str = "examples/Cassie/urdf/cassie_v2.urdf"
    simulate_perception: bool = False
    visualize: bool = True


class CassieFootstepControllerEnvironment(Diagram):

    def __init__(self, params: CassieFootstepControllerEnvironmentOptions):
        super().__init__()
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
        self.nq = self.controller_plant.num_positions()
        self.nv = self.controller_plant.num_velocities()

        builder = DiagramBuilder()
        self.controller = MpfcOscDiagram(
            self.controller_plant,
            params.osc_gains_yaml,
            params.mpfc_gains_yaml,
            params.osqp_options_yaml
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
        self.plant_visualizer = PlantVisualizer(params.urdf) if params.visualize else None

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
            builder.Connect(
                self.perception_module.get_output_port_robot_output(),
                self.controller.get_input_port_state()
            )
            builder.Connect(
                self.cassie_sim.get_output_port_depth_image(),
                self.perception_module.get_input_port_depth_image("pelvis_depth")
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
            # self.visualizer = self.cassie_sim.AddDrakeVisualizer(builder)

            if params.simulate_perception:
                self.grid_map_visualizer = GridMapVisualizer(
                    self.plant_visualizer.get_meshcat(), 30.0, ["elevation"]
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
            else:
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
            'state': builder.ExportOutput(
                self.cassie_sim.get_output_port_state(),
                'x, u, t'
            ),
            'lcmt_cassie_out': builder.ExportOutput(
                self.cassie_sim.get_output_port_cassie_out(),
                'lcmt_cassie_out'
            ),
        }

        if self.params.simulate_perception:
            output_port_indices['height_map'] = builder.ExportOutput(
                self.perception_module.get_output_port_elevation_map(),
                'elevation_map'
            )
        else:
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
        return footstep_controller

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
