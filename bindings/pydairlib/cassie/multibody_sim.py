import numpy as np
import pydrake
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import DiscreteTimeDelay
from pydairlib import multibody
from pydairlib.common import FindResourceOrThrow
from pydairlib.common import plot_styler
from pydairlib.systems.robot_lcm_systems import RobotInputReceiver
from pydairlib.systems.robot_lcm_systems import RobotOutputSender
from pydairlib.systems.robot_lcm_systems import RobotOutputReceiver
from pydairlib.systems.primitives import SubvectorPassThrough
from pydairlib.cassie.cassie_utils import *
from pydairlib.lcm import *
# from pydairlib.systems.lcm import *
import dairlib
import drake


def main():

  builder = DiagramBuilder()
  time_step = 8e-5
  penetration_allowance = 1e-5
  publish_rate = 2000
  initial_time = 30.0
  end_time = initial_time + 1.0
  rt_rate = 1.0
  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
  addCassieMultibody(plant, scene_graph, True, "examples/Cassie/urdf/cassie_v2.urdf", True, True)
  plant.set_penetration_allowance(penetration_allowance)
  plant.Finalize()

  lcm = pydrake.lcm.DrakeLcm()
  input_sub = pydrake.systems.lcm.LcmSubscriberSystem.Make(channel='CASSIE_INPUT', lcm_type=dairlib.lcmt_robot_input,
                                                           lcm=lcm, use_cpp_serializer=False)
  import pdb; pdb.set_trace()
  builder.AddSystem(input_sub)
  input_receiver = RobotInputReceiver(plant)
  builder.AddSystem(input_receiver)
  passthrough = builder.AddSystem(SubvectorPassThrough(input_receiver.get_output_port(0).size(), 0,
                                                       plant.get_actuation_input_port().size()))
  discrete_time_delay = builder.AddSystem(DiscreteTimeDelay(1 / publish_rate, 0, plant.num_actuators()))
  state_pub = pydrake.systems.lcm.LcmPublisherSystem.Make(channel='CASSIE_STATE_SIMULATION',
                                                          lcm_type=dairlib.lcmt_robot_output,
                                                          lcm=lcm)
  state_sender = RobotOutputSender(plant, True)
  builder.AddSystem(state_sender)

  # import pdb; pdb.set_trace()
  # sensor_aggregator = builder.AddSystem(AddImuAndAggregator(builder, plant, passthrough.get_output_port()))
  sensor_pub = pydrake.systems.lcm.LcmPublisherSystem.Make(channel='CASSIE_STATE_SIMULATION',
                                                           lcm_type=dairlib.lcmt_robot_output,
                                                           lcm=lcm)

  builder.Connect(input_sub.get_output_port(), input_receiver.get_input_port())
  # builder.Connect(input_sub, input_receiver)
  builder.Connect(input_receiver, passthrough)
  builder.Connect(passthrough.get_output_port(),
                  discrete_time_delay.get_input_port())
  builder.Connect(discrete_time_delay.get_output_port(),
                  plant.get_actuation_input_port())
  builder.Connect(plant.get_state_output_port(),
                  state_sender.get_input_port_state())
  builder.Connect(discrete_time_delay.get_output_port(),
                  state_sender.get_input_port_effort())
  builder.Connect(state_sender, state_pub)
  builder.Connect(
    plant.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(plant.get_source_id().value()))
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port())
  builder.Connect(sensor_aggregator.get_output_port(),
                  sensor_pub.get_input_port())
  diagram = builder.Build()
  diagram_context = diagram.CreateDefaultContext()
  diagram_context.EnableCaching()

  plant_context = plant.CreateDefaultContext()
  x_init = np.load('initial_trajectories.npy')
  plant.SetPositionsAndVelocities(plant_context, x_init)
  diagram_context.SetTime(initial_time)
  simulator = Simulator(diagram, diagram_context)
  simulator.set_publish_every_time_step(True)
  simulator.set_publish_at_initialization(False)
  simulator.set_target_realtime_rate(rt_rate)
  simulator.Initialize()
  simulator.AdvanceTo(end_time)


if __name__ == '__main__':
  main()
