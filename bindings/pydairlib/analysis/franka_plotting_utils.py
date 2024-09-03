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
from pydrake.all import MultibodyPlant, Parser, RigidTransform, RotationMatrix, RollPitchYaw, \
    FindResourceOrThrow

franka_urdf = FindResourceOrThrow(
    "drake/manipulation/models/franka_description/urdf/panda_arm.urdf")
end_effector_model = "examples/jacktoy/urdf/end_effector_full.urdf"
object_model = "examples/jacktoy/urdf/jack.sdf"
tool_attachment_frame = np.array([0, 0, 0.107])
franka_origin = np.array([0, 0, 0])

franka_default_channels = \
    {'FRANKA_STATE': dairlib.lcmt_robot_output,
     'FRANKA_STATE_SIMULATION': dairlib.lcmt_robot_output,
    #  'OBJECT_STATE': dairlib.lcmt_object_state,
     'OBJECT_STATE_SIMULATION': dairlib.lcmt_object_state,
    #  'OBJECT_STATE_SIMULATION': dairlib.lcmt_object_state,
    #  'FRANKA_INPUT': dairlib.lcmt_robot_input,
     'OSC_FRANKA': dairlib.lcmt_robot_input,
    #  'FRANKA_INPUT_ECHO': dairlib.lcmt_robot_input,
     'C3_TRAJECTORY_ACTOR_CURR_PLAN': dairlib.lcmt_timestamped_saved_traj,
     'C3_TRAJECTORY_OBJECT_CURR_PLAN': dairlib.lcmt_timestamped_saved_traj,
     'C3_TRAJECTORY_ACTOR_BEST_PLAN': dairlib.lcmt_timestamped_saved_traj,
     'C3_TRAJECTORY_OBJECT_BEST_PLAN': dairlib.lcmt_timestamped_saved_traj,
     'C3_DEBUG_BEST': dairlib.lcmt_c3_output,
     'C3_DEBUG_CURR': dairlib.lcmt_c3_output,
     'C3_ACTUAL': dairlib.lcmt_c3_state,
     'C3_TARGET': dairlib.lcmt_c3_state,
     'OSC_DEBUG_FRANKA': dairlib.lcmt_osc_output,
     'RADIO': dairlib.lcmt_radio_out,
     'SAMPLE_COSTS': dairlib.lcmt_timestamped_saved_traj,
     'CURR_AND_BEST_SAMPLE_COSTS': dairlib.lcmt_timestamped_saved_traj,
     'IS_C3_MODE': dairlib.lcmt_timestamped_saved_traj,
     'CONTACT_RESULTS': drake.lcmt_contact_results_for_viz,
     'DYNAMICALLY_FEASIBLE_CURR_PLAN': dairlib.lcmt_timestamped_saved_traj,
     'DYNAMICALLY_FEASIBLE_BEST_PLAN': dairlib.lcmt_timestamped_saved_traj,
     'C3_FORCES_CURR': dairlib.lcmt_c3_forces}
     


def make_plant_and_context():
    builder = DiagramBuilder()

    franka_plant = MultibodyPlant(0.0)
    franka_parser = Parser(franka_plant)
    franka_parser.AddModels(franka_urdf)
    end_effector_index = \
        franka_parser.AddModels(end_effector_model)[0]
    T_EE_W =  RigidTransform(RotationMatrix(RollPitchYaw(3.1415, 0, 0)),tool_attachment_frame)

    X_F_W =  RigidTransform(RotationMatrix(), franka_origin)

    franka_plant.WeldFrames(franka_plant.world_frame(),
                            franka_plant.GetFrameByName("panda_link0"),
                            X_F_W)
    franka_plant.WeldFrames(franka_plant.GetFrameByName("panda_link7"),
                            franka_plant.GetFrameByName("end_effector_base",
                                                        end_effector_index),
                            T_EE_W)

    object_plant = MultibodyPlant(0.0)
    object_parser = Parser(object_plant)
    object_parser.AddModels(object_model)

    franka_plant.Finalize()
    object_plant.Finalize()
    return (franka_plant, franka_plant.CreateDefaultContext(), object_plant,
            object_plant.CreateDefaultContext())
