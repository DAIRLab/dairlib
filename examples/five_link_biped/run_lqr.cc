#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/signal_logger.h"

#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "dairlib/lcmt_fsm_out.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/robot_lcm_systems.h"

#include "common/find_resource.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/goldilocks_models/file_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "examples/five_link_biped/walking_fsm.h"
#include "systems/controllers/hybrid_lqr.h"
#include "systems/framework/lcm_driven_loop.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"

using drake::multibody::Body;

using drake::AutoDiffXd;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

DEFINE_double(gravity, 9.81, "Gravity acceleration constant");
DEFINE_double(mu, 0.7, "The static coefficient of friction");
DEFINE_string(channel_x, "RABBIT_STATE",
              "Channel to publish/receive state from simulation");
DEFINE_string(channel_u, "RABBIT_INPUT",
              "Channel to publish/receive control efforts from simulation");
DEFINE_double(publish_rate, 2000, "Publishing frequency (Hz)");
DEFINE_double(buffer_time, 0.0,
              "Time around nominal impact time to apply "
              "heuristic efforts");
DEFINE_bool(naive, true,
            "Set to true if using the naive approach to hybrid lqr");
DEFINE_bool(contact_driven, true,
            "Set to true if want to use contact_driven fsm");
DEFINE_bool(recalculateP, false,
            "Set to true if necessary to recalculate P(t) - for new trajs");
DEFINE_bool(recalculateL, false,
            "Set to true if necessary to recalculate L(t) - for new trajs");
DEFINE_double(time_offset, 0.0, "offset added to FSM switching");
DEFINE_double(init_fsm_state, 0, "Initial FSM state.");
DEFINE_string(trajectory_name, "",
              "Filename for the trajectory that contains"
              " the initial state.");
DEFINE_string(folder_path, "",
              "Folder path for the folder that contains the "
              "saved trajectory");

namespace dairlib {

using drake::AutoDiffVecXd;
using drake::multibody::Frame;
using multibody::GetBodyIndexFromName;
using multibody::WorldPointEvaluator;
using systems::SubvectorPassThrough;

namespace examples {
namespace five_link_biped {
namespace hybrid_lqr {

const std::string channel_u = "RABBIT_INPUT";

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(0.0);
  SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
  Parser parser(&plant, &scene_graph);
  std::string full_name =
      FindResourceOrThrow("examples/five_link_biped/five_link_biped.urdf");
  parser.AddModelFromFile(full_name);
  plant.mutable_gravity_field().set_gravity_vector(-FLAGS_gravity *
                                                   Eigen::Vector3d::UnitZ());
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  plant.Finalize();
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      std::make_unique<MultibodyPlant<AutoDiffXd>>(plant);

  std::cout << "folder path: " << FLAGS_folder_path << std::endl;
  std::cout << "trajectory name: " << FLAGS_trajectory_name << std::endl;

  const LcmTrajectory& loaded_traj =
      LcmTrajectory(FLAGS_folder_path + FLAGS_trajectory_name);
  for (const auto& name : loaded_traj.getTrajectoryNames()) {
    std::cout << name << std::endl;
  }
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nx = nq + nv;
  int nu = plant.num_actuators();

  int num_modes = loaded_traj.getTrajectoryNames().size();
  std::vector<shared_ptr<PiecewisePolynomial<double>>> state_trajs;
  std::vector<shared_ptr<PiecewisePolynomial<double>>> input_trajs;
  for (int mode = 0; mode < num_modes; ++mode) {
    const LcmTrajectory::Trajectory& state_and_input =
        loaded_traj.getTrajectory("walking_trajectory_x_u" +
                                  std::to_string(mode));
    state_trajs.push_back(std::make_shared<PiecewisePolynomial<double>>(
        PiecewisePolynomial<double>::CubicHermite(
            state_and_input.time_vector, state_and_input.datapoints.topRows(nx),
            state_and_input.datapoints.topRows(2 * nx).bottomRows(nx))));
    input_trajs.push_back(std::make_shared<PiecewisePolynomial<double>>(
        PiecewisePolynomial<double>::FirstOrderHold(
            state_and_input.time_vector,
            state_and_input.datapoints.bottomRows(nu))));
  }

  vector<WorldPointEvaluator<double>> contact_evals;
  vector<WorldPointEvaluator<AutoDiffXd>> contact_evals_ad;

  WorldPointEvaluator<double> left_foot = WorldPointEvaluator(
      plant, VectorXd::Zero(3), plant.GetFrameByName("left_foot"));
  WorldPointEvaluator<double> right_foot = WorldPointEvaluator(
      plant, VectorXd::Zero(3), plant.GetFrameByName("right_foot"));
  WorldPointEvaluator<AutoDiffXd> left_foot_ad = WorldPointEvaluator(
      *plant_ad, VectorXd::Zero(3), plant_ad->GetFrameByName("left_foot"));
  WorldPointEvaluator<AutoDiffXd> right_foot_ad = WorldPointEvaluator(
      *plant_ad, VectorXd::Zero(3), plant_ad->GetFrameByName("left_foot"));

  // Contact modes go l_foot, r_foot, l_foot
  contact_evals.push_back(left_foot);
  contact_evals.push_back(right_foot);
  contact_evals.push_back(left_foot);
  contact_evals_ad.push_back(left_foot_ad);
  contact_evals_ad.push_back(right_foot_ad);
  contact_evals_ad.push_back(left_foot_ad);

  MatrixXd Q = 1 * MatrixXd::Identity(nx, nx);
  Q.block(0, 0, nq, nq) = 100 * MatrixXd::Identity(nq, nq);
  MatrixXd Qf = Q;
  MatrixXd R = 0.01 * MatrixXd::Identity(nu, nu);

  std::vector<double> impact_times(contact_evals.size() * 2);
  impact_times[0] = state_trajs[0]->start_time();
  impact_times[1] = state_trajs[0]->end_time();
  impact_times[2] = state_trajs[1]->start_time();
  impact_times[3] = state_trajs[1]->end_time();
  impact_times[4] = state_trajs[2]->start_time();
  impact_times[5] = state_trajs[2]->end_time();

  // Create Leaf Systems
  drake::lcm::DrakeLcm lcm;
  auto fsm = builder.AddSystem<WalkingFiniteStateMachine>(
      plant, impact_times[1], impact_times[3], FLAGS_time_offset,
      FLAGS_contact_driven, FLAGS_init_fsm_state);

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  auto contact_results_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", &lcm));
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          channel_u, &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto fsm_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_fsm_out>(
          "FSM", &lcm, TriggerTypeSet({TriggerType::kForced})));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

  auto start = std::chrono::high_resolution_clock::now();
  auto lqr = builder.AddSystem<systems::HybridLQRController>(
      plant, *plant_ad, contact_evals, contact_evals_ad, state_trajs,
      input_trajs, Q, R, Qf, FLAGS_buffer_time, FLAGS_naive, FLAGS_folder_path,
      FLAGS_recalculateP, FLAGS_recalculateL);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Took " << elapsed.count() << "s to create LQR controller"
            << std::endl;
  //  auto fsm_logger =
  //            drake::systems::LogOutput(fsm->get_output_port(0), &builder);
  //  fsm_logger->set_forced_publish_only();

  // ******End of leaf system initialization*******

  builder.Connect(lqr->get_output_port(0), command_sender->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0), lqr->get_input_port(0));
  builder.Connect(state_receiver->get_output_port(0),
                  fsm->get_state_input_port());
  builder.Connect(contact_results_sub->get_output_port(),
                  fsm->get_contact_input_port());
  builder.Connect(contact_results_sub->get_output_port(),
                  lqr->get_contact_input_port());
  builder.Connect(fsm->get_fsm_output_port(), lqr->get_fsm_input_port());
  builder.Connect(fsm->get_lcm_output_port(), fsm_pub->get_input_port());
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // Create the diagram and context
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, std::move(diagram), state_receiver, FLAGS_channel_x, true);
  loop.Simulate();
  drake::log()->info("controller started");

  return 0;
}

}  // namespace hybrid_lqr
}  // namespace five_link_biped
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::five_link_biped::hybrid_lqr::doMain(argc, argv);
}
