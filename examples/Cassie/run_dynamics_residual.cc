#include <memory>
#include <set>
#include <string>
#include <vector>

#include <gflags/gflags.h>

#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/lcm_contact_mode_translator.h"
#include "systems/dynamics_residual_system.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/connect_lcm_scope.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace dairlib {

using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using Eigen::Matrix3d;
using Eigen::Vector3d;

using std::set;
using std::string;
using std::vector;

// Simulation parameters.
DEFINE_string(address, "127.0.0.1", "IPv4 address to receive on.");
DEFINE_int64(port, 25001, "Port to receive on.");
DEFINE_double(pub_rate, 0.02, "Network LCM pubishing period (s).");

DEFINE_double(tau, 0.01, "Residual estimator time constant.");

DEFINE_string(state_channel_name, "CASSIE_STATE_DISPATCHER",
              "The name of the lcm channel containing lcmt_robot_output");
DEFINE_string(
    contact_channel_name, "CASSIE_GM_CONTACT_DISPATCHER",
    "The name of the lcm channel containing lcmt_contact_results_for_viz");

// Cassie model paramter
DEFINE_bool(floating_base, true, "Fixed or floating base model");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");
  DiagramBuilder<double> builder;

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, FLAGS_floating_base /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant.Finalize();

  // Evaluators for fourbar linkages
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);

  // Evaluators for contact points
  multibody::KinematicEvaluatorSet<double> left_contact_evaluator(plant);
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  left_contact_evaluator.add_evaluator(&left_toe_evaluator);
  left_contact_evaluator.add_evaluator(&left_heel_evaluator);
  left_contact_evaluator.add_evaluator(&left_loop);
  left_contact_evaluator.add_evaluator(&right_loop);

  multibody::KinematicEvaluatorSet<double> right_contact_evaluator(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Vector3d::Zero(), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Matrix3d::Identity(),
      Vector3d::Zero(), {0, 1, 2});
  right_contact_evaluator.add_evaluator(&right_toe_evaluator);
  right_contact_evaluator.add_evaluator(&right_heel_evaluator);
  right_contact_evaluator.add_evaluator(&left_loop);
  right_contact_evaluator.add_evaluator(&right_loop);

  multibody::KinematicEvaluatorSet<double> double_contact_evaluator(plant);
  double_contact_evaluator.add_evaluator(&right_toe_evaluator);
  double_contact_evaluator.add_evaluator(&right_heel_evaluator);
  double_contact_evaluator.add_evaluator(&left_toe_evaluator);
  double_contact_evaluator.add_evaluator(&left_heel_evaluator);
  double_contact_evaluator.add_evaluator(&left_loop);
  double_contact_evaluator.add_evaluator(&right_loop);

  std::vector<multibody::KinematicEvaluatorSet<double>*> evaluators{
      &left_contact_evaluator, &right_contact_evaluator,
      &double_contact_evaluator};

  // State subscriber/receiver (will drive loop)
  drake::lcm::Subscriber<dairlib::lcmt_robot_output> state_sub(
      &lcm_local, FLAGS_state_channel_name);

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  // Contact subscriber system
  auto contact_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<drake::lcmt_contact_results_for_viz>(
          FLAGS_contact_channel_name, &lcm_local));

  // Contact mode translator
  set<string> left_contact_list{"toe_left"};
  set<string> right_contact_list{"toe_right"};
  set<string> double_contact_list{"toe_left", "toe_right"};
  vector<set<string>> mode_contact_list{left_contact_list, right_contact_list,
                                        double_contact_list};
  auto contact_mode_translator =
      builder.AddSystem<systems::LcmContactModeTranslator>(mode_contact_list);

  // Dynamics residual system
  auto residual_system = builder.AddSystem<systems::DynamicResidualSystem>(
      plant, evaluators, FLAGS_tau);

  drake::systems::lcm::ConnectLcmScope(
      residual_system->get_residual_output_port(), "DYNAMICS_RESIDUAL",
      &builder, &lcm_local);

  // Connect systems
  builder.Connect(*contact_sub, *contact_mode_translator);
  builder.Connect(contact_mode_translator->get_output_port(0),
                  residual_system->get_mode_port());
  builder.Connect(state_receiver->get_output_port(),
                  residual_system->get_robot_output_port());

  // Create the diagram, simulator, and context.
  auto owned_diagram = builder.Build();
  const auto& diagram = *owned_diagram;
  drake::systems::Simulator<double> simulator(std::move(owned_diagram));
  auto& diagram_context = simulator.get_mutable_context();

  auto& state_receiver_context =
      diagram.GetMutableSubsystemContext(*state_receiver, &diagram_context);

  // Wait for the first message.
  drake::log()->info("Waiting for first lcmt_robot_output");

  LcmHandleSubscriptionsUntil(&lcm_local,
                              [&]() { return state_sub.count() > 0; });

  // Initialize the context based on the first message.
  const double t0 = state_sub.message().utime * 1e-6;
  diagram_context.SetTime(t0);

  auto& state_value = state_receiver->get_input_port(0).FixValue(
      &state_receiver_context, state_sub.message());

  drake::log()->info("dynamics residual started");
  while (true) {
    // Wait for an lcmt_cassie_out message.
    state_sub.clear();
    LcmHandleSubscriptionsUntil(&lcm_local,
                                [&]() { return state_sub.count() > 0; });
    // Write the input message into the context and advance.
    state_value.GetMutableData()->set_value(state_sub.message());
    const double time = state_sub.message().utime * 1e-6;

    // Check if we are very far ahead or behind
    // (likely due to a restart of the driving clock)
    if (time > simulator.get_context().get_time() + 1.0 ||
        time < simulator.get_context().get_time() - 1.0) {
      std::cout << "Residual time is " << simulator.get_context().get_time()
                << ", but stepping to " << time << std::endl;
      std::cout << "Difference is too large, resetting dispatcher time."
                << std::endl;
      simulator.get_mutable_context().SetTime(time);
      simulator.Initialize();
    }

    simulator.AdvanceTo(time);

    // Manual discrete variable update
    std::unique_ptr<drake::systems::DiscreteValues<double>> update =
      diagram.AllocateDiscreteVariables();
    update->SetFrom(diagram_context.get_mutable_discrete_state());
    diagram.CalcDiscreteVariableUpdates(diagram_context, update.get());
    diagram_context.get_mutable_discrete_state().SetFrom(*update);
    // Force-publish via the diagram
    diagram.Publish(diagram_context);
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
