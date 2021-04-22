import sys

import lcm
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from process_koopman_lcm_log import *
import pathlib
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import DiagramBuilder
import pydairlib.lcm_trajectory
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow

def main():

    builder = DiagramBuilder()
    plant, _  =AddMultibodyPlantSceneGraph(builder, 0.0)

    Parser(plant).AddModelFromFile(
        FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf"))

    plant.mutable_gravity_field().set_gravity_vector(-9.81*np.array([0,0,1]))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.Finalize()

    pos_map = pydairlib.multibody.makeNameToPositionsMap(plant)
    vel_map = pydairlib.multibody.makeNameToVelocitiesMap(plant)
    act_map = pydairlib.multibody.makeNameToActuatorsMap(plant)

    try:
        logfile_name = sys.argv[1]
    except:
        logfile_name = logfile_name = input("PLEASE ENTER LOG FILE NAME: ")
        pass

    log = lcm.EventLog(logfile_name, "r")
    robot_out_channel = "PLANAR_STATE"
    mpc_channel = "KOOPMAN_MPC_OUT"
    osc_channel = "PLANAR_INPUT"
    osc_debug_channel = "OSC_DEBUG_WALKING"

    mpc_output, x, u_meas, t_x, u, t_u, contact_forces, contact_info_locs, \
    t_contact_info, osc_debug, fsm, t_controller_switch, t_pd, kp, kd, \
    robot_out, osc_output, full_log = process_mpc_log(log, pos_map, vel_map,
        act_map, robot_out_channel, mpc_channel, osc_channel, osc_debug_channel)


if __name__ == "__main__":
    main()