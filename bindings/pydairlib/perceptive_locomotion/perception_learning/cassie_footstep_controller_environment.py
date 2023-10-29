from dataclasses import dataclass
from os import path
from typing import Dict, Union

import numpy as np

from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.perceptive_locomotion.controllers import MpfcOscDiagram, Stance
from pydairlib.perceptive_locomotion.simulators import HikingSimDiagram
from pydairlib.perceptive_locomotion.perception_learning.height_map_server \
    import HeightMapServer
from pydairlib.multibody import SquareSteppingStoneList

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.all import (
    Diagram,
    Context,
    Simulator,
    InputPort,
    OutputPort,
    DiagramBuilder,
    InputPortIndex,
    OutputPortIndex,
    ConstantVectorSource,
)

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"


class InitialConditionsServer:
    def __init__(self, fname=path.join(
        perception_learning_base_folder,
        'tmp/initial_conditions.npz'
    )):
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


@dataclass
class CassieFootstepControllerEnvironmentOptions:
    terrain: Union[str, SquareSteppingStoneList] = path.join(
        perception_learning_base_folder,
        'params/terrain.yaml'
    )
    rgdb_extrinsics_yaml: str = path.join(
        perception_learning_base_folder,
        'params/rgbd_extrinsics.yaml'
    )
    osc_gains_yaml: str = path.join(
        perception_learning_base_folder,
        'params/osc_gains.yaml'
    )
    mpfc_gains_yaml: str = path.join(
        perception_learning_base_folder,
        'params/mpfc_gains.yaml'
    )
    osqp_options_yaml: str = path.join(
        perception_learning_base_folder,
        'params/osqp_options_osc.yaml'
    )
    urdf: str = "examples/Cassie/urdf/cassie_v2.urdf"
    visualize: bool = True


class CassieFootstepControllerEnvironment(Diagram):

    def __init__(self, params: CassieFootstepControllerEnvironmentOptions):
        super().__init__()

        self.height_map_server = HeightMapServer(
            params.terrain,
            params.urdf
        )

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
        builder.Connect(
            self.cassie_sim.get_output_port_state_lcm(),
            self.controller.get_input_port_state(),
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

        self.visualizer = None
        if params.visualize:
            self.visualizer = self.cassie_sim.AddDrakeVisualizer(builder)

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
        robot_output = self.get_output_port_by_name('state').Eval(context)
        fsm = int(
            self.get_output_port_by_name('fsm').Eval(context)[0]
        )
        stance = Stance.kLeft if (fsm == 0 or fsm == 3) else Stance.kRight

        return self.height_map_server.get_heightmap_3d(
            robot_output[:self.nq + self.nv],
            stance,
            center
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
