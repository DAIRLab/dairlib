#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

#include "attic/multibody/multibody_solvers.h"
#include "attic/multibody/utility_systems.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/controllers/constrained_lqr_controller.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "examples/Cassie/cassie_utils.h"

#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

namespace dairlib {

using std::map;
using std::string;
using std::vector;
using std::unique_ptr;
using std::make_unique;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::multibody::AddFlatTerrainToWorld;
using drake::solvers::MathematicalProgramResult;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using dairlib::systems::SubvectorPassThrough;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::PositionSolver;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::multibody::StateToOutputVectorSystem;
using dairlib::systems::ConstrainedLQRController;

using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::Multiplexer;
using dairlib::systems::OutputVector;

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

// Cassie model paramter
DEFINE_bool(floating_base, false, "Fixed or floating base model");

// LCM channels
DEFINE_string(state_channel, "CASSIE_STATE",
              "LCM channel for receiving the state");
DEFINE_string(input_channel, "CASSIE_INPUT",
              "LCM channel for receiving the motor inputs");

/*
 * Function to run collisionDetect and return a ContactInfo object for the four
 * contact points in Cassie.
 * @param tree The RigidBodyTree containing the cassie model and a ground plane
 * box.
 * @param q0 The Cassie position configuration for which contact information is
 * to be computed.
 */
ContactInfo ComputeCassieContactInfo(const RigidBodyTree<double>& tree,
                                     const VectorXd& q0) {
  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;
  KinematicsCache<double> k_cache = tree.doKinematics(q0);

  // The full collisionDetect solution.
  const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
      k_cache, phi_total, normal_total, xA_total, xB_total, idxA_total,
      idxB_total);

  const int world_ind = GetBodyIndexFromName(tree, "world");
  const int toe_left_ind = GetBodyIndexFromName(tree, "toe_left");
  const int toe_right_ind = GetBodyIndexFromName(tree, "toe_right");

  // Extracting information into the four contacts.
  VectorXd phi(4);
  Matrix3Xd normal(3, 4), xA(3, 4), xB(3, 4);
  vector<int> idxA(4), idxB(4);

  int k = 0;
  for (unsigned i = 0; i < idxA_total.size(); ++i) {
    int ind_a = idxA_total.at(i);
    int ind_b = idxB_total.at(i);
    if ((ind_a == world_ind && ind_b == toe_left_ind) ||
        (ind_a == world_ind && ind_b == toe_right_ind) ||
        (ind_a == toe_left_ind && ind_b == world_ind) ||
        (ind_a == toe_right_ind && ind_b == world_ind)) {
      xA.col(k) = xA_total.col(i);
      xB.col(k) = xB_total.col(i);
      idxA.at(k) = idxA_total.at(i);
      idxB.at(k) = idxB_total.at(i);
      ++k;
    }
  }

  ContactInfo contact_info = {xB, idxB};
  return contact_info;
}

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::unique_ptr<RigidBodyTree<double>> tree;
  if (FLAGS_floating_base) {
    tree = makeCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf",
                                 drake::multibody::joints::kQuaternion);
    // Adding a terrain for the floating base version.
    AddFlatTerrainToWorld(tree.get(), 100, 0.1);
  } else {
    tree = makeCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf",
                                 drake::multibody::joints::kFixed);
  }

  const int num_positions = tree->get_num_positions();
  const int num_velocities = tree->get_num_velocities();
  const int num_states = num_positions + num_velocities;
  const int num_efforts = tree->get_num_actuators();

  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  if (FLAGS_simulation_type != "timestepping") {
    FLAGS_dt = 0.0;
  }

  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(
      std::move(tree), FLAGS_dt);

  // Setting contact parameters
  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);

  // Computing the fixed point.

  // Initializing the desired state.
  map<string, int> position_map =
      plant->get_rigid_body_tree().computePositionNameToIndexMap();
  // Vector containing indices of joints that need to be fixed.
  vector<int> fixed_joints;
  // Fixed joints map that is needed to add the constraint through the solver.
  map<int, double> fixed_joints_map;

  VectorXd x0 = VectorXd::Zero(num_states);

  for (auto it = position_map.begin(); it != position_map.end(); ++it) {
    std::cout << it->first << " " << it->second << std::endl;
  }

  if (FLAGS_floating_base) {
    // desired x0
    x0(position_map.at("base_z")) = 3;
    x0(position_map.at("base_qw")) = 1.0;
    x0(position_map.at("hip_roll_left")) = 0.1;
    x0(position_map.at("hip_roll_right")) = -0.1;
    x0(position_map.at("hip_yaw_left")) = -0.01;
    x0(position_map.at("hip_yaw_right")) = 0.01;
    x0(position_map.at("hip_pitch_left")) = .269;
    x0(position_map.at("hip_pitch_right")) = .269;
    x0(position_map.at("knee_left")) = -.744;
    x0(position_map.at("knee_right")) = -.744;
    x0(position_map.at("ankle_joint_left")) = .81;
    x0(position_map.at("ankle_joint_right")) = .81;
    x0(position_map.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0(position_map.at("toe_right")) = -60.0 * M_PI / 180.0;

    // Fixed joints
    fixed_joints.push_back(position_map.at("base_qw"));
    fixed_joints.push_back(position_map.at("base_qx"));
    fixed_joints.push_back(position_map.at("base_qy"));
    fixed_joints.push_back(position_map.at("base_qz"));
    // fixed_joints.push_back(position_map.at("hip_roll_left"));
    // fixed_joints.push_back(position_map.at("hip_roll_right"));
    // fixed_joints.push_back(position_map.at("hip_pitch_left"));
    // fixed_joints.push_back(position_map.at("hip_pitch_right"));
    fixed_joints.push_back(position_map.at("hip_yaw_left"));
    fixed_joints.push_back(position_map.at("hip_yaw_right"));
    // fixed_joints.push_back(position_map.at("knee_left"));
    // fixed_joints.push_back(position_map.at("knee_right"));

  } else {
    x0(position_map.at("hip_roll_left")) = 0.2;
    x0(position_map.at("hip_roll_right")) = -0.1;
    x0(position_map.at("hip_yaw_left")) = 0.1;
    x0(position_map.at("hip_yaw_right")) = -0.2;
    x0(position_map.at("hip_pitch_left")) = -0.3;
    x0(position_map.at("hip_pitch_right")) = .269;
    x0(position_map.at("knee_left")) = -.344;
    x0(position_map.at("knee_right")) = -.644;
    x0(position_map.at("ankle_joint_left")) = .792;
    x0(position_map.at("ankle_joint_right")) = .792;
    x0(position_map.at("toe_left")) = -60 * M_PI / 180.0;
    x0(position_map.at("toe_right")) = -30 * M_PI / 180.0;

    // Desired fixed joints
    fixed_joints.push_back(position_map.at("hip_pitch_left"));
    fixed_joints.push_back(position_map.at("hip_pitch_right"));
  }

  // Initial positions
  VectorXd q0 = x0.head(num_positions);

  // Setting up the map using the fixed joints vector
  for (auto& ind : fixed_joints) {
    fixed_joints_map[ind] = x0(ind);
  }

  // Cassie contact info
  ContactInfo contact_info;

  int num_forces = plant->get_rigid_body_tree().getNumPositionConstraints();

  if (FLAGS_floating_base) {
    contact_info = ComputeCassieContactInfo(plant->get_rigid_body_tree(), q0);
    // Accounting for the contact forces as well
    num_forces += 3 * contact_info.num_contacts;
  }

  VectorXd u0 = VectorXd::Zero(num_efforts);
  VectorXd lambda0 = VectorXd::Zero(num_forces);

  // Fixed point solver
  unique_ptr<FixedPointSolver> fp_solver;

  if (FLAGS_floating_base) {
    fp_solver = make_unique<FixedPointSolver>(plant->get_rigid_body_tree(),
                                              contact_info, q0, u0);
    fp_solver->SetInitialGuess(q0, u0, lambda0);
    fp_solver->AddSpreadNormalForcesCost();
    fp_solver->AddUnitQuaternionConstraint(
        position_map["base_qw"], position_map["base_qx"],
        position_map["base_qy"], position_map["base_qz"]);
    fp_solver->AddFrictionConeConstraint(0.8);
    fp_solver->AddFixedJointsConstraint(fixed_joints_map);
    fp_solver->AddJointLimitConstraint(0.001);
  } else {
    fp_solver =
        make_unique<FixedPointSolver>(plant->get_rigid_body_tree(), q0, u0);
    fp_solver->SetInitialGuess(q0, u0, lambda0);
    fp_solver->AddFixedJointsConstraint(fixed_joints_map);
    fp_solver->AddJointLimitConstraint(0.001);
  }

  std::cout << "Solving" << std::endl;
  MathematicalProgramResult fp_program_result = fp_solver->Solve();

  // Don't proceed if the solver does not find the right solution
  if (!fp_program_result.is_success()) {
    std::cout << "Fixed point solver error: "
              << fp_program_result.get_solution_result() << std::endl;
    return 0;
  }

  // Fixed point results.
  VectorXd q = fp_solver->GetSolutionQ();
  VectorXd u = fp_solver->GetSolutionU();
  VectorXd lambda = fp_solver->GetSolutionLambda();

  if (!fp_solver->CheckConstraint(q, u, lambda)) {
    std::cout << "Constraints not satisfied." << std::endl;
    return 0;
  }

  std::cout << "Joint angles: " << std::endl << q << std::endl;
  std::cout << "Torques: " << std::endl << u << std::endl;
  std::cout << "Forces: " << std::endl << lambda << std::endl;

  // Position solver for the start position
  VectorXd x_start(num_states);

  if (FLAGS_floating_base) {
    x_start << q, VectorXd::Zero(num_velocities);
  } else {
    x_start = VectorXd::Zero(num_states);
    unique_ptr<PositionSolver> position_solver;
    position_solver = make_unique<PositionSolver>(plant->get_rigid_body_tree(),
                                                  x_start.head(num_positions));
    position_solver->SetInitialGuessQ(x_start.head(num_positions));
    position_solver->AddJointLimitConstraint(0.001);

    MathematicalProgramResult position_program_result =
        position_solver->Solve();

    if (!position_program_result.is_success()) {
      std::cout << "Position solver error: "
                << position_program_result.get_solution_result() << std::endl;
      return 0;
    }

    x_start.head(num_positions) = position_solver->GetSolutionQ();
  }

  // Creating a state publisher
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(
      plant->get_rigid_body_tree());
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          FLAGS_state_channel, lcm, 1.0 / FLAGS_publish_rate));
  builder.Connect(plant->state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());

  // Input publisher
  auto input_sender = builder.AddSystem<systems::RobotCommandSender>(
      plant->get_rigid_body_tree());
  auto input_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_input_channel, lcm, 1.0 / FLAGS_publish_rate));
  builder.Connect(input_sender->get_output_port(0),
                  input_pub->get_input_port());

  // Constrained LQR controller

  // Defining the Q and R matrix
  // Positions for which the cost need to be ignored.
  vector<string> cost_ignore_position;
  if (FLAGS_floating_base) {
    cost_ignore_position = {"base_x", "base_y", "base_z"};
    //"base_roll", "base_pitch", "base_yaw"};
  }
  MatrixXd Q, R;

  // Separate costs for the fixed and floating versions.
  if (FLAGS_floating_base) {
    // Floating base
    double qp_mult = 1.0;
    double qv_mult = 100.0;
    MatrixXd Q_p = MatrixXd::Identity(num_positions, num_positions) * qp_mult;
    MatrixXd Q_v = MatrixXd::Identity(num_velocities, num_velocities) * qv_mult;
    Q = MatrixXd::Zero(num_states, num_states);

    for (auto& position : cost_ignore_position) {
      Q_p(position_map.at(position), position_map.at(position)) = 0;
      Q_v(position_map.at(position), position_map.at(position)) = 0;
    }
    Q.block(0, 0, num_positions, num_positions) = Q_p;
    Q.block(num_positions, num_positions, num_velocities, num_velocities) = Q_v;

    double r_mult = 0.01;
    double r_toe_mult = 0.001;
    R = MatrixXd::Identity(num_efforts, num_efforts) * r_mult;
    R(8, 8) = R(8, 8) * r_toe_mult;
    R(9, 9) = R(9, 9) * r_toe_mult;
  } else {
    // Fixed base
    double q_mult = 1.0;
    double r_mult = 1.0;
    Q = MatrixXd::Identity(num_states, num_states) * q_mult;
    R = MatrixXd::Identity(num_efforts, num_efforts) * r_mult;
  }

  // Initializing the controller
  auto clqr_controller = builder.AddSystem<ConstrainedLQRController>(
      plant->get_rigid_body_tree(), q, u, lambda, Q, R, contact_info);

  // Passthrough to remove the timestamp from the clqr controller output.
  auto passthrough = builder.AddSystem<SubvectorPassThrough<double>>(
      clqr_controller->get_output_port_efforts().size(), 0,
      plant->get_input_port(0).size());

  // State connector to connect the state of the plant to the controller.
  auto state_to_output_vector = builder.AddSystem<StateToOutputVectorSystem>(
      num_positions, num_velocities, num_efforts);
  builder.Connect(plant->state_output_port(),
                  state_to_output_vector->get_input_port(0));
  builder.Connect(state_to_output_vector->get_output_port(0),
                  clqr_controller->get_input_port_info());
  builder.Connect(clqr_controller->get_output_port_efforts(),
                  input_sender->get_input_port(0));
  builder.Connect(clqr_controller->get_output_port_efforts(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  plant->actuator_command_input_port());

  // Building the diagram and starting the simulation.
  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant,
                                          &simulator.get_mutable_context());

  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state =
        context.get_mutable_continuous_state();
    state.SetFromVector(x_start);
  } else {
    drake::systems::BasicVector<double>& state =
        context.get_mutable_discrete_state(0);
    state.SetFromVector(x_start);
  }

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  simulator.StepTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
