/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "examples/kuka_iiwa_arm/kuka_torque_controller.h"

namespace dairlib {
namespace examples {
namespace kuka_iiwa_arm {

 using Eigen::Vector3d;
 using Eigen::VectorXd;

 using drake::geometry::SceneGraph;
 using drake::manipulation::kuka_iiwa::IiwaCommandReceiver;
 using drake::manipulation::kuka_iiwa::IiwaStatusSender;
 using drake::math::RigidTransform;
 using drake::math::RollPitchYaw;
 using drake::multibody::Joint;
 using drake::multibody::MultibodyPlant;
 using drake::multibody::RevoluteJoint;
 using drake::multibody::SpatialInertia;
 using drake::multibody::ModelInstanceIndex;
 using drake::systems::StateInterpolatorWithDiscreteDerivative;

int DoMain() {

  std::unique_ptr<MultibodyPlant<double>> owned_world_plant =
      std::make_unique<MultibodyPlant<double>>(0.0001);
  std::unique_ptr<MultibodyPlant<double>> owned_controller_plant =
      std::make_unique<MultibodyPlant<double>>();
  std::unique_ptr<drake::geometry::SceneGraph<double>> owned_scene_graph =
      std::make_unique<drake::geometry::SceneGraph<double>>();

  MultibodyPlant<double>* world_plant =
      owned_world_plant.get();
  MultibodyPlant<double>* controller_plant =
      owned_controller_plant.get();
  drake::geometry::SceneGraph<double>* scene_graph =
      owned_scene_graph.get();
  world_plant->RegisterAsSourceForSceneGraph(scene_graph);

  // Get the Iiwa model. TODO: grab this from pegged drake libraries
  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf";
  const std::string kuka_urdf = drake::FindResourceOrThrow(kModelPath);
  const auto X_WI = RigidTransform<double>::Identity();

  // Add the Iiwa model to the world model
  drake::multibody::Parser world_plant_parser(world_plant);
  const drake::multibody::ModelInstanceIndex iiwa_model =
      world_plant_parser.AddModelFromFile(kuka_urdf, "iiwa");
  world_plant->WeldFrames(
      owned_world_plant->world_frame(),
      owned_world_plant->GetFrameByName("iiwa_link_0", iiwa_model), X_WI);

  // Create and add a plant to the controller-specific model
  drake::multibody::Parser controller_plant_parser(controller_plant);
  const auto controller_iiwa_model = controller_plant_parser.AddModelFromFile(
      kuka_urdf, "iiwa");
  owned_controller_plant->WeldFrames(owned_controller_plant->world_frame(),
      owned_controller_plant->GetFrameByName("iiwa_link_0", controller_iiwa_model),
      X_WI);

  // Finalize the plants to begin adding them to a system
  owned_controller_plant->Finalize();
  world_plant->Finalize();

  const int num_iiwa_positions = world_plant->num_positions();
  const int num_iiwa_velocities = world_plant->num_velocities();

  drake::systems::DiagramBuilder<double> builder;
  builder.AddSystem(std::move(owned_world_plant));
  builder.AddSystem(std::move(owned_scene_graph));

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Create the command subscriber and status publisher.
  auto command_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      builder.AddSystem<IiwaCommandReceiver>(num_iiwa_positions);
  command_receiver->set_name("command_receiver");

  // LCM publisher system
  const double kIiwaLcmStatusPeriod = 0.005;
  auto iiwa_status_publisher = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, kIiwaLcmStatusPeriod /* publish period */));

  // Torque Controller-- includes virtual springs and damping.
  VectorXd stiffness, damping_ratio;

  // The virtual spring stiffness in Nm/rad.
  stiffness.resize(num_iiwa_positions);
  stiffness << 2, 2, 2, 2, 2, 2, 2;

  // A dimensionless damping ratio. See KukaTorqueController for details.
  damping_ratio.resize(num_iiwa_positions);
  damping_ratio.setConstant(1.0);
  auto iiwa_controller = builder.AddSystem<
      dairlib::systems::KukaTorqueController<double>>(
          std::move(owned_controller_plant), stiffness, damping_ratio);

  // Creating status sender
  auto iiwa_status = builder.AddSystem<IiwaStatusSender>(num_iiwa_positions);

  // Creating system to approximate desired state command from a discrete
  // derivative of the position command input port.
  auto desired_state_from_position = builder.AddSystem<
      drake::systems::StateInterpolatorWithDiscreteDerivative<double>>(
          num_iiwa_positions, world_plant->time_step());

  // Demuxing system state for status publisher
  auto demux = builder.AddSystem<drake::systems::Demultiplexer<double>>(
     2 * num_iiwa_positions, num_iiwa_positions);

  builder.Connect(command_sub->get_output_port(),
                  command_receiver->get_input_port());

  // Connecting iiwa input ports
  builder.Connect(iiwa_controller->get_output_port_control(),
                  world_plant->get_actuation_input_port(iiwa_model));

  // Interpolating desired positions into desired velocities for controller.
  builder.Connect(command_receiver->get_commanded_position_output_port(),
                  desired_state_from_position->get_input_port());

  // Connecting iiwa controller input ports
  builder.Connect(world_plant->get_state_output_port(iiwa_model),
                  iiwa_controller->get_input_port_estimated_state());
  builder.Connect(desired_state_from_position->get_output_port(),
                  iiwa_controller->get_input_port_desired_state());
  builder.Connect(command_receiver->get_commanded_torque_output_port(),
                  iiwa_controller->get_input_port_commanded_torque());

  // Demux is for separating q and v from state output port.
  builder.Connect(world_plant->get_state_output_port(iiwa_model),
                  demux->get_input_port(0));

  // Connecting outputs to iiwa state broadcaster
  builder.Connect(demux->get_output_port(0),
                  iiwa_status->get_position_measured_input_port());
  builder.Connect(demux->get_output_port(1),
                  iiwa_status->get_velocity_estimated_input_port());
  builder.Connect(command_receiver->get_output_port(0),
                  iiwa_status->get_position_commanded_input_port());
  builder.Connect(iiwa_controller->get_output_port_control(),
                  iiwa_status->get_torque_commanded_input_port());
  builder.Connect(iiwa_controller->get_output_port_control(),
                  iiwa_status->get_torque_measured_input_port());
  builder.Connect(world_plant->get_generalized_contact_forces_output_port(iiwa_model),
                  iiwa_status->get_torque_external_input_port());
  builder.Connect(iiwa_status->get_output_port(),
                  iiwa_status_publisher->get_input_port());

  // Connecting world to scene graph components for visualization
  builder.Connect(
      world_plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(world_plant->get_source_id().value()));
  builder.Connect(scene_graph->get_query_output_port(),
                  world_plant->get_geometry_query_input_port());

  drake::geometry::ConnectDrakeVisualizer(&builder, *scene_graph,
      scene_graph->get_pose_bundle_output_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  // Set the iiwa default joint configuration.
  drake::VectorX<double> q0_iiwa(num_iiwa_positions + num_iiwa_velocities);
  q0_iiwa << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*world_plant,
                                          &simulator.get_mutable_context());

  drake::systems::BasicVector<double>& state =
      context.get_mutable_discrete_state(0);
  state.SetFromVector(q0_iiwa);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;

}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::examples::kuka_iiwa_arm::DoMain();
}
