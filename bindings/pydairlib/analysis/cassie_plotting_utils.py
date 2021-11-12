# python imports
import lcm
import numpy as np

# dairlib imports
import dairlib
from pydairlib.cassie.cassie_utils import *
from pydairlib.common import FindResourceOrThrow
from pydarilib.common import plot_styler

# drake imports
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder

cassie_urdf = "examples/Cassie/urdf/cassie_v2.urdf"
cassie_urdf_no_springs = "examples/Cassie/urdf/cassie_fixed_springs.urdf"

# Class to easily convert list of lcmt_osc_tracking_data_t to numpy arrays
class lcmt_osc_tracking_data_t:
    def __init__(self):
        self.t = []
        self.y_dim = 0
        self.name = ""
        self.is_active = []
        self.y = []
        self.y_des = []
        self.error_y = []
        self.ydot = []
        self.ydot_des = []
        self.error_ydot = []
        self.yddot_des = []
        self.yddot_command = []
        self.yddot_command_sol = []

    def append(self, msg, t):
        self.t.append(t)
        self.is_active.append(msg.is_active)
        self.y.append(msg.y)
        self.y_des.append(msg.y_des)
        self.error_y.append(msg.error_y)
        self.ydot.append(msg.ydot)
        self.ydot_des.append(msg.ydot_des)
        self.error_ydot.append(msg.error_ydot)
        self.yddot_des.append(msg.yddot_des)
        self.yddot_command.append(msg.yddot_command)
        self.yddot_command_sol.append(msg.yddot_command_sol)

    def convertToNP(self):
        self.t = np.array(self.t)
        self.is_active = np.array(self.is_active)
        self.y = np.array(self.y)
        self.y_des = np.array(self.y_des)
        self.error_y = np.array(self.error_y)
        self.ydot = np.array(self.ydot)
        self.ydot_des = np.array(self.ydot_des)
        self.error_ydot = np.array(self.error_ydot)
        self.yddot_des = np.array(self.yddot_des)
        self.yddot_command = np.array(self.yddot_command)
        self.yddot_command_sol = np.array(self.yddot_command_sol)

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

    pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
    vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
    act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

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

        return np.array(t_x), np.array(q), np.array(v), np.array(u)


def process_effort_data(data):
    u = []
    t = []
    for msg in data:
        t.append(msg.utime / 1e6)
        u.append(msg.efforts)

    return np.array(t), np.array(u)


def process_default_channels(data, plant, state_channel, input_channel,
                             osc_debug_channel):

    t_x, q, v, u_meas = process_state_channel(data[state_channel], plant)
    t_u, u = process_effort_data(data[input_channel])

    return {'t_x': t_x,
            'q': q,
            'v': v,
            'u_meas': u_meas,
            't_u': t_u,
            'u': u}
