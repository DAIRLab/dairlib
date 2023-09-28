from dataclasses import dataclass
from typing import Dict
from os import path
import numpy as np

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

from pydrake.multibody.plant import MultibodyPlant
from pydrake.geometry.all import MeshcatVisualizer, Meshcat

from pydairlib.common import FindResourceOrThrow
from pydairlib.cassie.cassie_utils import AddCassieMultibody
from pydairlib.perceptive_locomotion.controllers import MpfcOscDiagram
from pydairlib.perceptive_locomotion.simulators import HikingSimDiagram

perception_learning_base_folder = \
    "bindings/pydairlib/perceptive_locomotion/perception_learning"


@dataclass
class CassieFootstepControllerEnvironmentOptions:
    terrain_yaml: str = path.join(
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

        builder = DiagramBuilder()
        self.controller = MpfcOscDiagram(
            self.controller_plant,
            params.osc_gains_yaml,
            params.mpfc_gains_yaml,
            params.osqp_options_yaml
        )
        self.cassie_sim = HikingSimDiagram(
            params.terrain_yaml,
            params.rgdb_extrinsics_yaml
        )
        self.radio_source = ConstantVectorSource(np.zeros(18, ))
        builder.AddSystem(self.controller)
        builder.AddSystem(self.cassie_sim)
        builder.AddSystem(self.radio_source)
        builder.Connect(
            self.cassie_sim.get_output_port_state(),
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

    def get_output_port_by_name(self, name: str) -> InputPort:
        assert (name in self.output_port_indices)
        return self.get_output_port(self.output_port_indices[name])


if __name__ == '__main__':
    opts = CassieFootstepControllerEnvironmentOptions()
    env = CassieFootstepControllerEnvironment(opts)

    builder = DiagramBuilder()
    builder.AddSystem(env)
