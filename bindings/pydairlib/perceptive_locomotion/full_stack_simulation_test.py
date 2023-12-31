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
    ZeroOrderHold,
)

from pydairlib.perceptive_locomotion.diagrams import (
    AlipMPFCDiagram,
    MpfcOscDiagramInputType,
)

from pydairlib.perceptive_locomotion.systems. \
    cassie_footstep_controller_environment import (
    CassieFootstepControllerEnvironmentOptions,
    CassieFootstepControllerEnvironment,
)

from pydairlib.systems.system_utils import DrawAndSaveDiagramGraph

import numpy as np


def main():
    sim_params = CassieFootstepControllerEnvironmentOptions()
    sim_params.visualize = True
    sim_params.controller_input_type = MpfcOscDiagramInputType.kLcmtAlipMpcOutput

    sim_env = CassieFootstepControllerEnvironment(sim_params)
    controller = AlipMPFCDiagram(
        sim_env.controller_plant, sim_params.mpfc_gains_yaml, 1/30
    )


if __name__ == '__main__':
    main()