  #include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/math/autodiff.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/controllers/constrained_lqr_controller.h"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"

namespace dairlib {

// Simulation parameters.
DEFINE_double(height, .7, "The height to balance at");
DEFINE_double(Q_scale, 1, "Gain for Q");
DEFINE_double(Q_xy, 1, "Gain for Q");
DEFINE_double(R_toe_scale, 1, "Gain for R diagonal toe elements");

DEFINE_double(publish_rate, 1000, "Publishing frequency (Hz)");

// Cassie model paramter
DEFINE_bool(floating_base, true, "Fixed or floating base model");

// LCM channels
DEFINE_string(state_channel, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving the state");
DEFINE_string(input_channel, "CASSIE_INPUT",
              "LCM channel for receiving the motor inputs");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using Eigen::VectorXd;

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build <double> and <AutoDiffXd> plants
  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, FLAGS_spring_model, false);
  plant.Finalize();

  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
    drake::systems::System<double>::ToAutoDiffXd(plant);

  // Get a nominal fixed point
  VectorXd q, u, lambda;

  // Use fixed springs model to find a good fixed point
  double mu_fp = 0;
  double min_normal_fp = 70;
  double toe_spread = .2;
  if (FLAGS_floating_base) {
    CassieFixedPointSolver(plant, FLAGS_height, mu_fp, min_normal_fp,
        true, toe_spread, &q, &u, &lambda);  
  } else {
    CassieFixedBaseFixedPointSolver(plant, &q, &u, &lambda);
  }

  drake::systems::DiagramBuilder<double> builder;
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
    "udpm://239.255.76.67:7667?ttl=0");


  const std::string channel_x = FLAGS_state_channel;
  const std::string channel_u = FLAGS_input_channel;

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);
  builder.Connect(*state_sub, *state_receiver);

  // Create command sender.
  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          channel_u, lcm, 1.0 / FLAGS_publish_rate));
  auto command_sender =
      builder.AddSystem<systems::RobotCommandSender>(plant);

  builder.Connect(*command_sender, *command_pub);

  // *************Build the controller*************  
  multibody::KinematicEvaluatorSet<AutoDiffXd> evaluators(*plant_ad);

  // Add loop closures  
  auto left_loop = LeftLoopClosureEvaluator(*plant_ad);
  auto right_loop = RightLoopClosureEvaluator(*plant_ad);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);


  // Add contact points. Tangent basis must now be active, to treat these
  // as true constraints.
  auto left_toe = LeftToeFront(*plant_ad);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(*plant_ad,
      left_toe.first, left_toe.second, Eigen::Matrix3d::Identity(),
      Eigen::Vector3d::Zero(), {1,2});

  auto right_toe = RightToeFront(*plant_ad);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(*plant_ad,
      right_toe.first, right_toe.second, Eigen::Matrix3d::Identity(),
      Eigen::Vector3d::Zero(), {1,2});

  auto left_heel = LeftToeRear(*plant_ad);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(*plant_ad,
      left_heel.first, left_heel.second);

  auto right_heel = RightToeRear(*plant_ad);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(*plant_ad,
      right_heel.first, right_heel.second);

  if (FLAGS_floating_base) {
    evaluators.add_evaluator(&left_toe_evaluator);
    evaluators.add_evaluator(&right_toe_evaluator);
    evaluators.add_evaluator(&left_heel_evaluator);
    evaluators.add_evaluator(&right_heel_evaluator);
  }


  // Create a context
  VectorXd xul(plant.num_positions() + plant.num_velocities()
      + plant.num_actuators() + evaluators.count_full());
  xul << q, VectorXd::Zero(plant.num_velocities()), u, lambda;
  AutoDiffVecXd xul_ad = drake::math::InitializeAutoDiff(xul);

  AutoDiffVecXd x_ad = xul_ad.head(plant.num_positions()
      + plant.num_velocities());
  AutoDiffVecXd u_ad = xul_ad.segment(plant.num_positions()
      + plant.num_velocities(), plant.num_actuators());

  auto context_autodiff =
      multibody::CreateContext<AutoDiffXd>(*plant_ad, x_ad, u_ad);

  // controller gains
  Eigen::MatrixXd Q =
      Eigen::MatrixXd::Zero(plant.num_positions() + plant.num_velocities(),
          plant.num_positions() + plant.num_velocities());

  Q << FLAGS_Q_scale * 10 * Eigen::MatrixXd::Identity(plant.num_positions(),
          plant.num_positions()),
       Eigen::MatrixXd::Zero(plant.num_positions(), plant.num_velocities()),
       Eigen::MatrixXd::Zero(plant.num_velocities(), plant.num_positions()),
       FLAGS_Q_scale * 1 * Eigen::MatrixXd::Identity(plant.num_velocities(),
          plant.num_velocities());

  Q(4,4) *= FLAGS_Q_xy;
  Q(5,5) *= FLAGS_Q_xy;


  Eigen::MatrixXd R =  
      Eigen::MatrixXd::Identity(plant.num_actuators(), plant.num_actuators());
  R(8,8) *= FLAGS_R_toe_scale;
  R(9,9) *= FLAGS_R_toe_scale;

  auto controller = builder.AddSystem<systems::ConstrainedLQRController>(
      evaluators, *context_autodiff, lambda, Q, R);

  builder.Connect(*state_receiver, *controller);
  builder.Connect(*controller, *command_sender);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  {
    // Wait to receive first message
    drake::lcm::DrakeLcm lcm_wait("udpm://239.255.76.67:7667?ttl=0");
    drake::log()->info("Waiting for first lcmt_robot_output");
    drake::lcm::Subscriber<dairlib::lcmt_robot_output> input_sub(&lcm_wait,
        channel_x);
    LcmHandleSubscriptionsUntil(&lcm_wait, [&]() {
      return input_sub.count() > 0; });
  }
  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("controller started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
