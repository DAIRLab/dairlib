import numpy as np
import pydrake
from pydairlib.cassie.cassie_utils import *
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import DiscreteTimeDelay
import pydairlib.multibody
from pydairlib.common import FindResourceOrThrow
from pydairlib.common import plot_styler
from pydairlib.systems.robot_lcm_systems import RobotInputReceiver
from pydairlib.systems.robot_lcm_systems import RobotOutputSender
from pydairlib.systems.robot_lcm_systems import RobotOutputReceiver
import dairlib


def main():
  builder = DiagramBuilder()
  time_step = 8e-5
  penetration_allowance = 1e-5
  publish_rate = 2000
  initial_time = 30.0
  end_time = initial_time + 1.0
  rt_rate = 1.0
  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step)
  pydairlib.cassie.cassie_utils.addCassieMultibody(plant, scene_graph, True,
                                                   "examples/Cassie/urdf/cassie_v2.urdf", True, True)
  plant.set_penetration_allowance(penetration_allowance)
  plant.Finalize()

  lcm = pydrake.lcm.DrakeLcm()
  input_sub = pydrake.systems.lcm.LcmSubscriberSystem.Make(channel='CASSIE_INPUT', lcm_type=dairlib.lcmt_robot_input,
                                                           lcm=lcm)
  input_receiver = RobotInputReceiver(plant)
  # passthrough = builder.AddSystem(pydrake.systems.lcm.LcmSubscriberSystem(plant))
  discrete_time_delay = builder.AddSystem(DiscreteTimeDelay(1 / publish_rate, 0, plant.num_actuators()))
  state_pub = pydrake.systems.lcm.LcmPublisherSystem.Make(channel='CASSIE_STATE_SIMULATION', lcm_type=dairlib.lcmt_robot_output,
                                                           lcm=lcm)
  state_sender = RobotOutputSender(plant, True)
  import pdb; pdb.set_trace()
  # state_sender = builder.AddSystem(RobotOutputSender(plant, true))

  sensor_aggregator = builder.AddImuAndAggregator(plant, passthrough.get_output_port())
  sensor_pub = builder.AddSystem(LcmPublisherSystem("CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate))

  builder.Connect(input_sub, input_receiver)
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
