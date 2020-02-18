#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "attic/multibody/multibody_solvers.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/lcm/drake_lcm.h"

namespace dairlib {

using dairlib::multibody::PositionSolver;
using dairlib::systems::SubvectorPassThrough;
using drake::solvers::MathematicalProgramResult;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

// Simulation parameters.
DEFINE_double(timestep, 1e-4, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 2e-4,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant",
              "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3,
              "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");
DEFINE_double(publish_rate, 1000, "Publishing frequency (Hz)");
DEFINE_bool(
    publish_state, true,
    "Publish state PLANAR_STATE (set to false when running w/dispatcher");
DEFINE_string(state_channel_name, "PLANAR_STATE",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_bool(publish_output, true, "Publish simulated PLANAR_OUTPUT");

DEFINE_double(target_realtime_rate, 1.0,
              "The target realtime rate for the simulation");

// Planar Walker inital positions
DEFINE_double(init_height, 0.855, "Initial height of the pelvis");
DEFINE_double(left_hip_pin, 0.58234,
              "Initial angle between torso and left upper link");
DEFINE_double(left_knee_pin, -1.16473,
              "Initial angle between left upper link and left lower link");
DEFINE_double(left_ankle, 0.58234,
              "Initial angle between left lower link and left foot");
DEFINE_double(right_hip_pin, 0.58234,
              "Initial angle between torso and right upper link");
DEFINE_double(right_knee_pin, -1.16473,
              "Initial angle between right upper link and right lower link");
DEFINE_double(right_ankle, 0.58234,
              "Initial angle between right lower link and right foot");

DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time of simulation");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/PlanarWalker/PlanarWalkerWithTorsoAndFeet.urdf",
      drake::multibody::joints::kFixed, tree.get());

  for (int i = 0; i < tree.get()->get_num_positions(); i++) {
    std::cout << tree.get()->get_position_name(i) << std::endl;
  }

  const double terrain_size = 100;
  const double terrain_depth = 0.20;
  drake::multibody::AddFlatTerrainToWorld(tree.get(), terrain_size,
                                          terrain_depth);

  drake::systems::DiagramBuilder<double> builder;

  if (!FLAGS_publish_output && !FLAGS_publish_state) {
    throw std::logic_error(
        "Must publish either via PLANAR_OUTPUT or PLANAR_STATE");
  }

  if (FLAGS_simulation_type != "timestepping") FLAGS_dt = 0.0;
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(
      std::move(tree), FLAGS_dt);

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Note: this sets identical contact parameters across all object pairs:
  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);

  // Create input receiver
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          "PLANAR_INPUT", lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(
      plant->get_rigid_body_tree());
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant->get_input_port(0).size());

  // To get rid of the timestamp which is at the tail of a timestapedVector
  builder.Connect(input_sub->get_output_port(),
                  input_receiver->get_input_port(0));
  builder.Connect(input_receiver->get_output_port(0),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(), plant->get_input_port(0));

  if (FLAGS_publish_state) {
    // Create state publisher
    auto state_sender = builder.AddSystem<systems::RobotOutputSender>(
        plant->get_rigid_body_tree());
    auto state_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
            FLAGS_state_channel_name, lcm, 1.0 / FLAGS_publish_rate));
    builder.Connect(plant->state_output_port(),
                    state_sender->get_input_port_state());
    builder.Connect(state_sender->get_output_port(0),
                    state_pub->get_input_port());
  }

  drake::lcm::DrakeLcm drake_lcm;
  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(
      plant->get_rigid_body_tree(), &drake_lcm);
  // Raw state vector to visualizer.
  builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant,
                                          &simulator.get_mutable_context());

  Eigen::VectorXd x0 =
      Eigen::VectorXd::Zero(plant->get_rigid_body_tree().get_num_positions() +
                            plant->get_rigid_body_tree().get_num_velocities());

  std::map<std::string, int> map =
      plant->get_rigid_body_tree().computePositionNameToIndexMap();
  x0(map.at("left_ankle")) = FLAGS_left_ankle;
  x0(map.at("left_knee_pin")) = FLAGS_left_knee_pin;
  x0(map.at("left_hip_pin")) = FLAGS_left_hip_pin;
  x0(map.at("right_ankle")) = FLAGS_right_ankle;
  x0(map.at("right_knee_pin")) = FLAGS_right_knee_pin;
  x0(map.at("right_hip_pin")) = FLAGS_right_hip_pin;

  std::vector<int> fixed_joints;
  // fixed_joints.push_back(map.at("knee_left"));
  // fixed_joints.push_back(map.at("knee_right"));

  // Set the initial height of the robot so that it's above the ground.
  x0(1) = FLAGS_init_height;

  Eigen::VectorXd q0 =
      x0.head(plant->get_rigid_body_tree().get_num_positions());
  PositionSolver position_solver(plant->get_rigid_body_tree(), q0);
  position_solver.SetInitialGuessQ(q0);

  // Creating the map for the fixed joints constraint
  std::map<int, double> fixed_joints_map;
  for (auto& ind : fixed_joints) {
    fixed_joints_map[ind] = x0(ind);
  }

  position_solver.AddFixedJointsConstraint(fixed_joints_map);

  MathematicalProgramResult program_result = position_solver.Solve();

  if (!program_result.is_success()) {
    std::cout << "Solver error: " << program_result.get_solution_result()
              << std::endl;
    return 0;
  }

  q0 = position_solver.GetSolutionQ();
  x0.head(plant->get_rigid_body_tree().get_num_positions()) = q0;

  std::cout << q0 << std::endl;

  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state =
        context.get_mutable_continuous_state();
    std::cout << "Continuous " << state.size() << std::endl;
    x0[1] = FLAGS_init_height;
    state.SetFromVector(x0);
  } else {
    std::cout << "ngroups " << context.num_discrete_state_groups() << std::endl;
    drake::systems::BasicVector<double>& state =
        context.get_mutable_discrete_state(0);
    std::cout << "Discrete " << state.size() << std::endl;
    state.SetFromVector(x0);
  }

  // int num_u = plant->get_num_actuators();
  // auto zero_input = Eigen::MatrixXd::Zero(num_u, 1);
  // context.FixInputPort(0, zero_input);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_end_time);
  // simulator.AdvanceTo(.01);
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
