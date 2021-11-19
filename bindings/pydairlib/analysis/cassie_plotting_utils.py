# python imports
import lcm
import numpy as np
import matplotlib.pyplot as plt

# dairlib imports
import dairlib
from pydairlib.cassie.cassie_utils import addCassieMultibody


# drake imports
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder

cassie_urdf = "examples/Cassie/urdf/cassie_v2.urdf"
cassie_urdf_no_springs = "examples/Cassie/urdf/cassie_fixed_springs.urdf"


cassie_default_channels = \
    {'CASSIE_STATE_SIMULATION': dairlib.lcmt_robot_output,
     'CASSIE_STATE_DISPATCHER': dairlib.lcmt_robot_output,
     'CASSIE_INPUT': dairlib.lcmt_robot_input,
     'CASSIE_OUTPUT': dairlib.lcmt_cassie_out,
     'OSC_DEBUG_STANDING': dairlib.lcmt_osc_output,
     'OSC_DEBUG_WALKING': dairlib.lcmt_osc_output,
     'OSC_DEBUG_JUMPING': dairlib.lcmt_osc_output,
     'OSC_DEBUG_RUNNING': dairlib.lcmt_osc_output}


def make_plant_and_context(floating_base=True, springs=True):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    if (springs):
        addCassieMultibody(plant, scene_graph,
                           floating_base, cassie_urdf, True, True)
    else:
        addCassieMultibody(plant, scene_graph,
                           floating_base, cassie_urdf_no_springs, False, True)

    plant.Finalize()
    return plant, plant.CreateDefaultContext()
