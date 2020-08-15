#include <string>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib::goldilocks_models {

using std::cout;
using std::endl;
using std::to_string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;

using systems::OutputVector;

using multibody::JwrtqdotToJwrtv;

//DEFINE_int32(iter, 29, "The iteration # of the model parameter that you use");

DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(
    channel_fsm_t, "FSM_T",
    "LCM channel for receiving fsm and time of latest liftoff event. ");
DEFINE_string(channel_y, "MPC_OUTPUT",
              "The name of the channel which publishes command");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_feedback(0.0);
  addCassieMultibody(&plant_feedback, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_feedback.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_controls(0.0);
  addCassieMultibody(&plant_controls, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_controls.Finalize();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_feedback);

  // Create Lcm subscriber for fsm and latest lift off time
  auto fsm_and_liftoff_time_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<drake::lcmt_drake_signal>(
          FLAGS_channel_fsm_t, &lcm_local));

  // Create mpc traj publisher
  auto traj_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_trajectory_block>(
          FLAGS_channel_y, &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  // Create optimal rom trajectory generator
  // TODO(yminchen): need to centralize the fsm state
  int left_stance_state = 0;
  int right_stance_state = 1;
  double stride_period = 0.37;  // TODO(yminchen): this value should change
  std::vector<int> ss_fsm_states = {left_stance_state, right_stance_state};
  auto rom_planner = builder.AddSystem<OptimalRomPlanner>(
      plant_feedback, plant_controls, ss_fsm_states, stride_period);
  builder.Connect(state_receiver->get_output_port(0),
                  rom_planner->get_input_port_state());
  builder.Connect(fsm_and_liftoff_time_receiver->get_output_port(),
                  rom_planner->get_input_port_fsm_and_lo_time());
  builder.Connect(rom_planner->get_output_port(0),
                  traj_publisher->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  loop.Simulate();

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
