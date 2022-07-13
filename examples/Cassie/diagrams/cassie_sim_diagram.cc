
#include "cassie_sim_diagram.h"

#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <iostream>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/geared_motor.h"
#include "systems/primitives/radio_parser.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/geometry/drake_visualizer.h"
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
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::math::RotationMatrix;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

CassieSimDiagram::CassieSimDiagram(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
    const std::string& urdf, bool visualize, double mu, double stiffness,
    double dissipation_rate) {
  DiagramBuilder<double> builder;
  scene_graph_ = builder.AddSystem<SceneGraph>();
  scene_graph_->set_name("scene_graph");

  plant_ = builder.AddSystem(std::move(plant));
  AddCassieMultibody(plant_, scene_graph_, true, urdf, true, true);
  multibody::AddFlatTerrain(plant_, scene_graph_, mu, mu,
                            Eigen::Vector3d(0, 0, 1), stiffness,
                            dissipation_rate);
  plant_->Finalize();

  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(*plant_);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant_->get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          actuator_update_rate, actuator_delay / actuator_update_rate,
          plant_->num_actuators() + 1);
  auto state_sender =
      builder.AddSystem<systems::RobotOutputSender>(*plant_, true);

  auto constant_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(10));
  auto input_zero_order_hold = builder.AddSystem<drake::systems::ZeroOrderHold>(
      0.001, plant_->num_actuators() + 1);
  sensor_aggregator_ = &AddImuAndAggregator(&builder, *plant_,
                                            constant_source->get_output_port());

  cassie_motor_ = &AddMotorModel(&builder, *plant);

  auto radio_parser = builder.AddSystem<systems::RadioParser>();

  // connect leaf systems
  builder.Connect(input_receiver->get_output_port(),
                  input_zero_order_hold->get_input_port());
  builder.Connect(input_zero_order_hold->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  cassie_motor_->get_input_port_command());
  builder.Connect(cassie_motor_->get_output_port(),
                  plant_->get_actuation_input_port());
  builder.Connect(plant_->get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(plant_->get_state_output_port(),
                  cassie_motor_->get_input_port_state());
  builder.Connect(cassie_motor_->get_output_port(),
                  state_sender->get_input_port_effort());
  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());
  builder.Connect(radio_parser->get_output_port(),
                  sensor_aggregator_->get_input_port_radio());

  builder.ExportInput(input_receiver->get_input_port(), "lcmt_robot_input");
  builder.ExportInput(radio_parser->get_input_port(), "raw_radio");
  builder.ExportOutput(state_sender->get_output_port(0), "lcmt_robot_output");
  builder.ExportOutput(sensor_aggregator_->get_output_port(0),
                       "lcmt_cassie_out");
  if (visualize) {
    DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph_);
  }
  builder.BuildInto(this);
  this->set_name("cassie_sim_diagram");
  DrawAndSaveDiagramGraph(*this);
  std::cout << "Built simulator" << std::endl;
}
}  // namespace examples
}  // namespace dairlib