# python imports
import lcm
import numpy as np
import matplotlib.pyplot as plt

# lcmtype imports
import dairlib
import drake

# drake imports
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.all import MultibodyPlant, Parser, RigidTransform

franka_urdf = "examples/franka/urdf/franka_no_collision.urdf"
tray_model = "examples/franka/urdf/tray.sdf"
franka_default_channels = \
  {'FRANKA_STATE': dairlib.lcmt_robot_output,
   # 'TRAY_STATE': dairlib.lcmt_robot_output,
   'FRANKA_INPUT': dairlib.lcmt_robot_input,
   'C3_TRAJECTORY_ACTOR': dairlib.lcmt_timestamped_saved_traj,
   'C3_TRAJECTORY_TRAY': dairlib.lcmt_timestamped_saved_traj,
   'OSC_DEBUG_FRANKA': dairlib.lcmt_osc_output,
   'RADIO': dairlib.lcmt_radio_out,
   'CONTACT_RESULTS': drake.lcmt_contact_results_for_viz}


def make_plant_and_context():
  builder = DiagramBuilder()

  franka_plant = MultibodyPlant(0.0)
  franka_parser = Parser(franka_plant)
  franka_parser.AddModelFromFile(franka_urdf)

  tray_plant = MultibodyPlant(0.0)
  tray_parser = Parser(tray_plant)
  tray_parser.AddModelFromFile(tray_model)

  franka_plant.WeldFrames(franka_plant.world_frame(), franka_plant.GetFrameByName("panda_link0"), RigidTransform())

  franka_plant.Finalize()
  tray_plant.Finalize()
  return franka_plant, franka_plant.CreateDefaultContext(), tray_plant, tray_plant.CreateDefaultContext()
