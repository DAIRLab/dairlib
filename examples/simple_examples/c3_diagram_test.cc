#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "systems/controllers/c3_controller.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {
namespace examples {

using drake::AutoDiffXd;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::SortedPair;
using drake::geometry::GeometryId;
using std::vector;

DEFINE_string(urdf, "examples/simple_examples/five_link_biped.urdf",
              "model urdf");
DEFINE_double(dt, 1e-3, "The step size to use for ");
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(start_time, 0.0, "Time to start the simulator at.");
DEFINE_double(sim_time, std::numeric_limits<double>::infinity(),
              "The length of time to run the simulation");

int DoMain(int argc, char* argv[]) {
//  DiagramBuilder<double> builder;
//  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(FLAGS_dt);
//  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
//  Parser parser(&plant, &scene_graph);
//  std::string full_name = FindResourceOrThrow(FLAGS_urdf);
//  parser.AddModelFromFile(full_name);
//  /// specific to this urdf (will change)
//  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
//                   drake::math::RigidTransform<double>());
//  ///
//
//  plant.Finalize();
//
//  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
//      std::make_unique<MultibodyPlant<AutoDiffXd>>(plant);
//  auto context = plant.CreateDefaultContext();
//  auto context_ad = plant_ad->CreateDefaultContext();
//
//
//  /// system specific?
//  const std::vector<GeometryId>& left_foot_geoms =
//      plant.GetCollisionGeometriesForBody(
//          plant.GetBodyByName("left_foot"));
//  const std::vector<GeometryId>& right_foot_geoms =
//      plant.GetCollisionGeometriesForBody(
//          plant.GetBodyByName("right_foot"));
//  const std::vector<GeometryId>& base_geoms =
//      plant.GetCollisionGeometriesForBody(
//          plant.GetBodyByName("base"));
//
//
//  std::vector<SortedPair<GeometryId>> contact_geoms;
//  contact_geoms.push_back(
//      SortedPair(base_geoms[0], right_foot_geoms[0]));
//  contact_geoms.push_back(
//      SortedPair(base_geoms[0], right_foot_geoms[0]));
//
//
//  int num_friction_directions = 2;
//  double mu = 0.8;
//  ///
//
//  ///param
//  int N = 5;
//  int n = plant.num_positions() + plant.num_velocities();
//  int m = 10;
//  int k = plant.num_actuators();
//  MatrixXd Qinit(n, n);
//  Qinit = MatrixXd::Identity(n,n);
//  MatrixXd Rinit(k, k);
//  Rinit = MatrixXd::Identity(k,k);
//  MatrixXd Uinit(n + m + k, n + m + k);
//  Uinit = MatrixXd::Identity(n+m+k,n+m+k);
//  MatrixXd Ginit(n + m + k, n + m + k);
//  Ginit = MatrixXd::Identity(n+m+k,n+m+k);
//  VectorXd xdesiredinit(n);
//  xdesiredinit = VectorXd::Zero(n);
//
//  std::vector<MatrixXd> Q(N + 1, Qinit);
//  std::vector<MatrixXd> R(N, Rinit);
//  std::vector<MatrixXd> U(N, Uinit);
//  std::vector<MatrixXd> G(N, Ginit);
//  std::vector<VectorXd> xdesired(N, xdesiredinit);
//
//  /*
//  systems::controllers::C3Controller controller(
//      plant, *context, *plant_ad, *context_ad, contact_geoms,
//      num_friction_directions, mu, Q, R, G, U, xdesired);
//  */
//
//  systems::controllers::C3Controller* controller =
//      builder.AddSystem<systems::controllers::C3Controller>(
//          plant, *context, *plant_ad, *context_ad, contact_geoms,
//          num_friction_directions, mu, Q, R, G, U, xdesired);
//
//  /// get rid of t in controller output port
//  auto passthrough = builder.AddSystem<systems::SubvectorPassThrough>(
//      controller->get_output_port(0).size(), 0,
//      plant.get_actuation_input_port().size());
//
//  /// conversion plant -> controller
//  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);
//  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
//
//  builder.Connect(controller->get_input_port_output(),
//                  passthrough->get_input_port());
//
//  builder.Connect(passthrough->get_output_port(),
//                  plant.get_actuation_input_port());
//
//  /*
//  builder.Connect(plant.get_state_output_port(),
//                  controller->get_input_port_config());
//*/
//
//  builder.Connect(plant.get_state_output_port(),
//                  state_sender->get_input_port());
//
//  builder.Connect(state_sender->get_output_port(),
//                  state_receiver->get_input_port());
//
//  builder.Connect(state_receiver->get_output_port(),
//                  controller->get_input_port());
//
//  builder.Connect(
//      plant.get_geometry_poses_output_port(),
//      scene_graph.get_source_pose_port(plant.get_source_id().value()));
//
//  builder.Connect(scene_graph.get_query_output_port(),
//                  plant.get_geometry_query_input_port());
//
//  auto diagram = builder.Build();
//  // Create a context for this system:
//  std::unique_ptr<Context<double>> diagram_context =
//      diagram->CreateDefaultContext();
//  diagram_context->EnableCaching();
//  diagram->SetDefaultContext(diagram_context.get());
//
//  // DrawAndSaveDiagramGraph(*diagram);
//
//  Simulator<double> simulator(*diagram, std::move(diagram_context));
//
//  simulator.set_publish_every_time_step(false);
//  simulator.set_publish_at_initialization(false);
//  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
//  simulator.Initialize();
//  simulator.AdvanceTo(FLAGS_start_time + FLAGS_sim_time);
//
//  return 0;
}

}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
