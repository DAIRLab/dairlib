#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "attic/multibody/multibody_solvers.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "examples/Cassie/cassie_utils.h"

#include "drake/systems/sensors/accelerometer.h"
#include "drake/systems/sensors/gyroscope.h"
#include "systems/sensors/sim_cassie_sensor_aggregator.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using dairlib::multibody::PositionSolver;
using drake::solvers::MathematicalProgramResult;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

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
DEFINE_bool(publish_state, true,
    "Publish state CASSIE_STATE (set to false when running w/dispatcher");
DEFINE_string(state_channel_name, "CASSIE_STATE",
              "The name of the lcm channel that sends Cassie's state");
DEFINE_bool(publish_cassie_output, true, "Publish simulated CASSIE_OUTPUT");

// Cassie model paramter
DEFINE_bool(floating_base, false, "Fixed or floating base model");
// Cassie inital positions
DEFINE_double(init_height, 1.05, "Initial height of the pelvis");
DEFINE_double(init_hip_pitch, .4, "Initial hip pitch angle");
DEFINE_double(init_knee, -1.0, "Initial knee joint position");
DEFINE_double(init_ankle, 1.3, "Initial ankle joint position");
DEFINE_double(init_toe, -1.5, "Initial toe joint position");

DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
    "End time of simulation");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::unique_ptr<RigidBodyTree<double>> tree;
  if (!FLAGS_floating_base){
    tree = makeCassieTreePointer();
  } else {
    tree = makeCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf",
                                 drake::multibody::joints::kQuaternion);
    const double terrain_size = 100;
    const double terrain_depth = 0.20;
    drake::multibody::AddFlatTerrainToWorld(
        tree.get(), terrain_size, terrain_depth);
  }

  // Add imu frame to Cassie's pelvis
  if (FLAGS_publish_cassie_output) addImuFrameToCassiePelvis(tree);

  drake::systems::DiagramBuilder<double> builder;

  if (!FLAGS_publish_cassie_output && !FLAGS_publish_state) {
    throw std::logic_error(
        "Must publish either via CASSIE_OUTPUT or CASSIE_STATE");
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
          "CASSIE_INPUT", lcm));
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

  // Create cassie output (containing simulated sensor) publisher
  if (FLAGS_publish_cassie_output) {
    auto cassie_sensor_aggregator =
        addImuAndAggregatorToSimulation(builder, plant, passthrough);
    auto cassie_sensor_pub =
        builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
            "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));
    builder.Connect(cassie_sensor_aggregator->get_output_port(0),
                    cassie_sensor_pub->get_input_port());
  }

  // Creates and adds LCM publisher for visualization.
  // builder.AddVisualizer(&lcm);
  // auto visualizer = builder.AddSystem<systems::DrakeVisualizer>
  //                   (plant->get_rigid_body_tree(), &lcm);
  // Raw state vector to visualizer.
  // builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant,
                                          &simulator.get_mutable_context());

  // drake::systems::Context<double>& sim_context =
      // simulator.get_mutable_context();
  // auto integrator =
  //     simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
  //         *diagram, FLAGS_timestep, &sim_context);
  // auto integrator =
  //   simulator.reset_integrator<drake::systems::RungeKutta3Integrator<double>>
  //   (*diagram, &sim_context);
  // integrator->set_maximum_step_size(FLAGS_timestep);

  Eigen::VectorXd x0 =
      Eigen::VectorXd::Zero(plant->get_rigid_body_tree().get_num_positions() +
                            plant->get_rigid_body_tree().get_num_velocities());

  std::map<std::string, int> map =
      plant->get_rigid_body_tree().computePositionNameToIndexMap();
  x0(map.at("hip_pitch_left")) = FLAGS_init_hip_pitch;
  x0(map.at("hip_pitch_right")) = FLAGS_init_hip_pitch;
  // x0(map.at("achilles_hip_pitch_left")) = -.44;
  // x0(map.at("achilles_hip_pitch_right")) = -.44;
  // x0(map.at("achilles_heel_pitch_left")) = -.105;
  // x0(map.at("achilles_heel_pitch_right")) = -.105;
  x0(map.at("knee_left")) = FLAGS_init_knee;
  x0(map.at("knee_right")) = FLAGS_init_knee;
  x0(map.at("ankle_joint_left")) = FLAGS_init_ankle;
  x0(map.at("ankle_joint_right")) = FLAGS_init_ankle;

  // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
  // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;

  // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
  // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;

  x0(map.at("toe_left")) = FLAGS_init_toe;
  x0(map.at("toe_right")) = FLAGS_init_toe;

  std::vector<int> fixed_joints;
  fixed_joints.push_back(map.at("hip_pitch_left"));
  fixed_joints.push_back(map.at("hip_pitch_right"));
  fixed_joints.push_back(map.at("knee_left"));
  fixed_joints.push_back(map.at("knee_right"));

  if (FLAGS_floating_base) {
    double quaternion_norm = x0.segment(3, 4).norm();
    if (quaternion_norm != 0)  // Unit Quaternion
      x0.segment(3, 4) = x0.segment(3, 4) / quaternion_norm;
    else  // in case the user enters 0-norm quaternion
      x0(3) = 1;
  }

  // Set the initial height of the robot so that it's above the ground.
  if (FLAGS_floating_base) {
    x0(2) = FLAGS_init_height;
  }

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
    std::cout << "Solver error: " << program_result.get_solution_result() << std::endl;
    return 0;
  }

  q0 = position_solver.GetSolutionQ();
  x0.head(plant->get_rigid_body_tree().get_num_positions()) = q0;

  std::cout << q0 << std::endl;

  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state =
        context.get_mutable_continuous_state();
    std::cout << "Continuous " << state.size() << std::endl;
    state.SetFromVector(x0);
    // state[4] = 1;
    // state[3] = 0;
    // state[4] = 0;
  } else {
    std::cout << "ngroups " << context.num_discrete_state_groups()
              << std::endl;
    drake::systems::BasicVector<double>& state =
        context.get_mutable_discrete_state(0);
    std::cout << "Discrete " << state.size() << std::endl;
    state.SetFromVector(x0);
    // state[4] = 1;
    // state[3] = 0;
    // state[4] = 0;
  }

  // int num_u = plant->get_num_actuators();
  // auto zero_input = Eigen::MatrixXd::Zero(num_u,1);
  // context.FixInputPort(0, zero_input);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_end_time);
  // simulator.AdvanceTo(.01);
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }