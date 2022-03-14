
#include "cassie_sim_diagram.h"

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/geared_motor.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

namespace dairlib {
namespace examples {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

CassieSimDiagram::CassieSimDiagram(const std::string& urdf, double mu,
                                   double stiffness, double dissipation_rate) {
  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = dt_;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  multibody::AddFlatTerrain(&plant, &scene_graph, mu, mu,
                            Eigen::Vector3d(0, 0, 1), stiffness,
                            dissipation_rate);

  addCassieMultibody(&plant, &scene_graph, true, urdf, true, true);
  plant.Finalize();

  // Create lcm systems.
//  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
//  auto input_sub =
//      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
//          FLAGS_channel_u, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          1e-3, FLAGS_actuator_delay * FLAGS_publish_rate,
          plant.num_actuators());
//  auto state_pub =
//      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
//          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(
      plant, true);

  // Contact Information
//  ContactResultsToLcmSystem<double>& contact_viz =
//      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
//  contact_viz.set_name("contact_visualization");
//  auto& contact_results_publisher = *builder.AddSystem(
//      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
//          "CASSIE_CONTACT_DRAKE", lcm, 1.0 / FLAGS_publish_rate));
//  contact_results_publisher.set_name("contact_results_publisher");

  // Sensor aggregator and publisher of lcmt_cassie_out
//  auto radio_sub =
//      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(
//          FLAGS_radio_channel, lcm));
  const auto& sensor_aggregator =
      AddImuAndAggregator(&builder, plant, passthrough->get_output_port());

//  auto sensor_pub =
//      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
//          "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));

  std::vector<double> omega_max = {303.687, 303.687, 136.136, 136.135, 575.958,
                                   303.687, 303.687, 136.136, 136.135, 575.958};

  auto cassie_motor = builder.AddSystem<systems::GearedMotor>(plant, omega_max);

  // connect leaf systems
//  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(input_receiver->get_output_port(),
                  cassie_motor->get_input_port_command());
  builder.Connect(cassie_motor->get_output_port(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(plant.get_state_output_port(),
                  cassie_motor->get_input_port_state());
  builder.Connect(discrete_time_delay->get_output_port(),
                  state_sender->get_input_port_effort());
//  builder.Connect(*state_sender, *state_pub);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
//  builder.Connect(plant.get_contact_results_output_port(),
//                  contact_viz.get_input_port(0));
//  builder.Connect(contact_viz.get_output_port(0),
//                  contact_results_publisher.get_input_port());
//  builder.Connect(radio_sub->get_output_port(),
//                  sensor_aggregator.get_input_port_radio());
//  builder.Connect(sensor_aggregator.get_output_port(0),
//                  sensor_pub->get_input_port());

  builder.ExportInput(input_receiver->get_input_port(),
                      "u");
  builder.ExportInput(sensor_aggregator.get_input_port(), "radio");
  builder.ExportOutput(state_sender->get_output_port(0));
  builder.ExportOutput(sensor_aggregator.get_output_port(0));

  auto diagram = builder.Build();
  diagram->set_name(("multibody_sim"));
}
}  // namespace examples
}  // namespace dairlib