# python imports
import lcm
import numpy as np
import matplotlib.pyplot as plt

# dairlib imports
import dairlib
from pydairlib.cassie.cassie_utils import *
from pydairlib.common import FindResourceOrThrow
from pydairlib.common import plot_styler
from pydairlib.multibody import makeNameToPositionsMap, makeNameToVelocitiesMap, makeNameToActuatorsMap
from osc_debug import lcmt_osc_tracking_data_t

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

def make_plant_and_context(floating=True, springs=True):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    if (springs):
        addCassieMultibody(plant, scene_graph, floating, cassie_urdf, True, True)
    else:
        addCassieMultibody(plant, scene_graph, floating, cassie_urdf_no_springs,
                           False, True)

    plant.Finalize()
    return plant, plant.CreateDefaultContext()


def process_state_channel(state_data, plant):
    t_x = []
    q = []
    u = []
    v = []

    pos_map = makeNameToPositionsMap(plant)
    vel_map = makeNameToVelocitiesMap(plant)
    act_map = makeNameToActuatorsMap(plant)

    for msg in state_data:
        q_temp = [[] for i in range(len(msg.position))]
        v_temp = [[] for i in range(len(msg.velocity))]
        u_temp = [[] for i in range(len(msg.effort))]
        for i in range(len(q_temp)):
            q_temp[pos_map[msg.position_names[i]]] = msg.position[i]
        for i in range(len(v_temp)):
            v_temp[vel_map[msg.velocity_names[i]]] = msg.velocity[i]
        for i in range(len(u_temp)):
            u_temp[act_map[msg.effort_names[i]]] = msg.effort[i]
        q.append(q_temp)
        v.append(v_temp)
        u.append(u_temp)
        t_x.append(msg.utime / 1e6)

        return {'t_x': np.array(t_x),
                'q': np.array(q),
                'v': np.array(v),
                'u': np.array(u)}


def process_effort_channel(data):
    u = []
    t = []
    for msg in data:
        t.append(msg.utime / 1e6)
        u.append(msg.efforts)

    return {'t_u': np.array(t), 'u': np.array(u)}


def process_osc_channel(data):
    t_osc = []
    osc_output = []
    osc_debug_tracking_datas = {}
    fsm = []

    for msg in data:
        t_osc.append(msg.utime / 1e6)
        osc_output.append(msg)
        num_osc_tracking_data = len(msg.tracking_data)
        for i in range(num_osc_tracking_data):
            if msg.tracking_data[i].name not in osc_debug_tracking_datas:
                osc_debug_tracking_datas[msg.tracking_data[i].name] = \
                    lcmt_osc_tracking_data_t()
            osc_debug_tracking_datas[msg.tracking_data[i].name].append(
                msg.tracking_data[i], msg.utime / 1e6)
        fsm.append(msg.fsm_state)

    for name in osc_debug_tracking_datas.keys():
        osc_debug_tracking_datas[name] = \
            osc_debug_tracking_datas[name].convertToNP()

    return {'t_osc': t_osc,
            'osc_output': osc_output,
            'osc_debug_tracking_datas': osc_debug_tracking_datas}


def load_default_channels(data, plant, state_channel, input_channel,
                          osc_debug_channel):

    robot_output = process_state_channel(data[state_channel], plant)
    robot_input = process_effort_channel(data[input_channel])
    osc_debug = process_osc_channel(data[osc_debug_channel])

    return robot_output, robot_input, osc_debug


def make_channel_plot(data_dictionary, time_key, keys_to_plot, legend_entries,
                      plot_labels, ps=plot_styler.PlotStyler()):
    legend = []
    for key in keys_to_plot:
        ps.plot(data_dictionary[time_key], data_dictionary[key])
        legend.extend(legend_entries[key])

    plt.legend(legend)
    plt.xlabel(plot_labels['xlabel'])
    plt.ylabel(plot_labels['ylabel'])
    plt.title(plot_labels['title'])
