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
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "attic/multibody/rigidbody_utils.h"
#include "dairlib/lcmt_fsm_out.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/robot_lcm_systems.h"

#include "common/find_resource.h"
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
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_string(channel_x, "RABBIT_STATE_SIMULATION",
              "Channel to publish/receive state from simulation");
DEFINE_double(publish_rate, 2000, "Publishing frequency (Hz)");
DEFINE_double(buffer_time, 0.0, "Time around nominal impact time to apply "
                                "heuristic efforts");
DEFINE_bool(naive, true,
            "Set to true if using the naive approach to hybrid lqr");
DEFINE_bool(minimal_coords, true,
            "Set to true if using minimal coords for constrained hybrid lqr");
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

using multibody::GetBodyIndexFromName;
using systems::SubvectorPassThrough;

namespace examples {
namespace five_link_biped {
namespace hybrid_lqr {

const std::string channel_u = "RABBIT_INPUT";

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  DiagramBuilder<double> builder;

  MultibodyPlant<double> plant(1e-5);
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
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      std::make_unique<MultibodyPlant<AutoDiffXd>>(plant);

  std::cout << "folder path: " << FLAGS_folder_path << std::endl;
  std::cout << "trajectory name: " << FLAGS_trajectory_name << std::endl;

  const LcmTrajectory& loaded_traj =
      LcmTrajectory(FLAGS_folder_path + FLAGS_trajectory_name);
  std::cout << "Saved trajectory names: " << std::endl;
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

  vector<multibody::ContactInfo<double>> contact_modes;
  vector<multibody::ContactInfo<AutoDiffXd>> contact_modes_ad;
  vector<const drake::multibody::Frame<double>*> l_foot_body_frame;
  vector<const drake::multibody::Frame<double>*> r_foot_body_frame;
  vector<const drake::multibody::Frame<AutoDiffXd>*> l_foot_body_frame_ad;
  vector<const drake::multibody::Frame<AutoDiffXd>*> r_foot_body_frame_ad;
  l_foot_body_frame.push_back(&(plant.GetBodyByName("left_foot").body_frame()));
  r_foot_body_frame.push_back(
      &(plant.GetBodyByName("right_foot").body_frame()));
  l_foot_body_frame_ad.push_back(
      &(plant_autodiff->GetBodyByName("left_foot").body_frame()));
  r_foot_body_frame_ad.push_back(
      &(plant_autodiff->GetBodyByName("right_foot").body_frame()));
  multibody::ContactInfo<double> l_foot_contact;
  multibody::ContactInfo<double> r_foot_contact;
  multibody::ContactInfo<AutoDiffXd> l_foot_contact_ad;
  multibody::ContactInfo<AutoDiffXd> r_foot_contact_ad;
  l_foot_contact.xA = VectorXd::Zero(3);
  l_foot_contact.xB = VectorXd::Zero(3);
  r_foot_contact.xA = VectorXd::Zero(3);
  r_foot_contact.xB = VectorXd::Zero(3);
  l_foot_contact.frameA = l_foot_body_frame;
  r_foot_contact.frameA = r_foot_body_frame;

  l_foot_contact_ad.xA = VectorXd::Zero(3);
  l_foot_contact_ad.xB = VectorXd::Zero(3);
  r_foot_contact_ad.xA = VectorXd::Zero(3);
  r_foot_contact_ad.xB = VectorXd::Zero(3);
  l_foot_contact_ad.frameA = l_foot_body_frame_ad;
  r_foot_contact_ad.frameA = r_foot_body_frame_ad;

  // Contact modes go l_foot, r_foot, l_foot
  contact_modes.push_back(l_foot_contact);
  contact_modes.push_back(r_foot_contact);
  contact_modes.push_back(l_foot_contact);
  contact_modes_ad.push_back(l_foot_contact_ad);
  contact_modes_ad.push_back(r_foot_contact_ad);
  contact_modes_ad.push_back(l_foot_contact_ad);

  MatrixXd Q = 1 * MatrixXd::Identity(nx, nx);
  Q.block(0, 0, nq, nq) = 100 * MatrixXd::Identity(nq, nq);
  MatrixXd Qf = Q;
  MatrixXd R = 0.01 * MatrixXd::Identity(nu, nu);

  std::vector<double> impact_times(contact_modes.size() * 2);
  impact_times[0] = state_trajs[0]->start_time();
  impact_times[1] = state_trajs[0]->end_time();
  impact_times[2] = state_trajs[1]->start_time();
  impact_times[3] = state_trajs[1]->end_time();
  impact_times[4] = state_trajs[2]->start_time();
  impact_times[5] = state_trajs[2]->end_time();

  // Create Leaf Systems
  // Create state receiver.
  // Create command sender.
  drake::lcm::DrakeLcm lcm;
  auto fsm = builder.AddSystem<WalkingFiniteStateMachine>(
      plant, impact_times[1], impact_times[3], FLAGS_time_offset,
      FLAGS_contact_driven, FLAGS_init_fsm_state);
  // Create state receiver.
  //  auto state_sub = builder.AddSystem(
  //      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x,
  //      lcm));

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
      plant, *plant_autodiff, contact_modes, contact_modes_ad, Q, R, Qf,
      FLAGS_buffer_time, state_trajs, input_trajs, impact_times,
      FLAGS_folder_path, FLAGS_naive, FLAGS_minimal_coords, FLAGS_recalculateP,
      FLAGS_recalculateL);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Took " << elapsed.count() << "s to create LQR controller"
            << std::endl;
  //  auto fsm_logger =
  //            drake::systems::LogOutput(fsm->get_output_port(0), &builder);
  //  fsm_logger->set_forced_publish_only();

  // ******End of leaf system initialization*******

  //  builder.Connect(state_sub->get_output_port(),
  //                  state_receiver->get_input_port(0));
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
  //  builder.Connect(fsm->get_output_port(0), lqr_cost->get_input_port(1));
  //  builder.Connect(state_receiver->get_output_port(0),
  //                  lqr_cost->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  //  builder.Connect(contact_receiver->get_output_port(0),
  //      )

  // Create the diagram and context
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  //  std::cout << "Built diagram" << std::endl;
  //  /// Use the simulator to drive at a fixed rate
  //  /// If set_publish_every_time_step is true, this publishes twice
  //  /// Set realtime rate. Otherwise, runs as fast as possible
  //  auto stepper = std::make_unique<drake::systems::Simulator<double>>(
  //      *diagram, std::move(context));
  //  stepper->set_publish_every_time_step(false);
  //  stepper->set_publish_at_initialization(false);
  //  stepper->set_target_realtime_rate(1.0);
  //  stepper->Initialize();
  //  std::cout << "Running simulation" << std::endl;
  //
  //  drake::log()->info("controller started");
  //  stepper->AdvanceTo(std::numeric_limits<double>::infinity());
  //  stepper->AdvanceTo(3.0);

  // Write all dataloggers to a CSV
  //  MatrixXd value_function = value_function_logger->data();
  //  MatrixXd estimated_cost = lqr_cost_logger->data();
  //  MatrixXd fsm_output = fsm_logger->data();

  //  goldilocks_models::writeCSV(
  //      "../projects/five_link_biped/hybrid_lqr/plotting/V.csv",
  //      fsm_output.transpose());
  //  goldilocks_models::writeCSV(
  //      "../projects/five_link_biped/hybrid_lqr/plotting/lqr.csv",
  //      estimated_cost.transpose());
  //  goldilocks_models::writeCSV("../projects/hybrid_lqr/plotting/inputs.csv",
  //                              input_matrix.transpose());
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
