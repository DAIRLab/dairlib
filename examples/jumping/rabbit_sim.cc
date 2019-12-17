#include <memory>
#include <string>
#include <map>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"

#include "multibody/multibody_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/gyroscope.h"

namespace dairlib {
using dairlib::systems::SubvectorPassThrough;
using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::solvers::MathematicalProgramResult;
using drake::systems::Simulator;
using drake::multibody::RevoluteJoint;
using drake::multibody::PrismaticJoint;

using drake::systems::Context;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::ContactResultsToLcmSystem;


// Simulation parameters.
DEFINE_double(timestep, 1e-4, "The simulator time step (s)");
DEFINE_double(x_initial, 0, "The initial x position of the torso");

DEFINE_bool(floating_base, true, "Fixed or floating base model");

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, false,
            "If 'true', the plant is modeled as a "
            "discrete system with periodic updates. "
            "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt,
              1e-4,
              "The step size to use for compliant, ignored for time_stepping)");

DEFINE_string(state_simulation_channel, "RABBIT_STATE_SIMULATION",
              "Channel to publish/receive state from simulation");
DEFINE_string(input_channel, "RABBIT_INPUT",
              "Channel to publish/receive inputs from controller");

DEFINE_bool(visualize, false, "Whether or not to connect to Drake visualizer");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  // drake::lcm::DrakeLcm lcm;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;

  MultibodyPlant<double>
      & plant = *builder.AddSystem<MultibodyPlant>(FLAGS_timestep);

  Parser parser(&plant, &scene_graph);
  std::string
      full_name = FindResourceOrThrow("examples/jumping/five_link_biped.urdf");
  parser.AddModelFromFile(full_name);
  plant.WeldFrames(
      plant.world_frame(),
      plant.GetFrameByName("base"),
      drake::math::RigidTransform<double>()
  );
  plant.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
  multibody::addFlatTerrain(&plant, &scene_graph, .8, .8); // Add ground

  plant.Finalize();

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_input_channel,
          lcm));

  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  builder.Connect(*input_sub, *input_receiver);

  // connect input receiver
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(),
      0,
      plant.get_actuation_input_port().size());

  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());

  // Create state publisher.
  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "RABBIT_STATE_SIMULATION",
          lcm,
          1.0 / 1000.0
      ));

  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);

  // connect state publisher
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());
  builder.Connect(plant.get_geometry_poses_output_port(),
                  scene_graph.get_source_pose_port(
                      plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  if (FLAGS_visualize) {
    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  }

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  Eigen::VectorXd x0(14);
  // x0  << 0, 0.7768, 0, -0.3112, -0.231, 0.427, 0.4689,
  x0 << FLAGS_x_initial, 0.778109, 0, -.3112, -.231, 0.427, 0.4689,
      0, 0, 0, 0, 0, 0, 0;
  plant.SetPositionsAndVelocities(&plant_context, x0);

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
