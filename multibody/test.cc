#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_solvers.h"
#include "multibody/rbt_utils.h"

namespace dairlib {

using std::cout;
using std::endl;
using std::vector;
using std::unique_ptr;
using std::move;
using std::map;
using std::string;

using Eigen::VectorXi;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::EigenSolver;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::MatrixX;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::BasicVector;
using drake::systems::ConstantVectorSource;
using drake::systems::DrakeVisualizer;
using drake::systems::Multiplexer;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::systems::RigidBodyPlant;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kFixed;
using drake::multibody::AddFlatTerrainToWorld;

using dairlib::buildCassieTree;
using dairlib::makeCassieTreePointer;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::multibody::PositionSolver;
using dairlib::multibody::ContactSolver;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::ContactInfo;

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 1.0, "The static coefficient of friction");
DEFINE_double(ud, 1.0, "The dynamic coefficient of friction");
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

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Variable that changes the base type
  bool fixed_base = false;

  drake::lcm::DrakeLcm lcm;
  RigidBodyTree<double> tree;

  std::string filename = "examples/Cassie/urdf/cassie_v2.urdf";
  unique_ptr<RigidBodyTree<double>> tree_ptr;

  if (fixed_base) {
    tree_ptr = makeCassieTreePointer(filename, kFixed);
    buildCassieTree(tree, filename, kFixed);
  } else {
    tree_ptr = makeCassieTreePointer(filename, kRollPitchYaw);
    buildCassieTree(tree, filename, kRollPitchYaw);
  }
  // Adding the ground
  AddFlatTerrainToWorld(&tree, 4, 0.05);
  AddFlatTerrainToWorld(tree_ptr.get(), 4, 0.05);

  if (FLAGS_simulation_type != "timestepping") {
    FLAGS_dt = 0.0;
  }

  int num_positions = tree.get_num_positions();
  int num_velocities = tree.get_num_velocities();
  int num_states = num_positions + num_velocities;
  int num_efforts = tree.get_num_actuators();
  int num_position_constraints = tree.getNumPositionConstraints();
  int num_contacts = 4;
  int num_forces = num_position_constraints + num_contacts * 3;

  drake::systems::DiagramBuilder<double> builder;

  // Change the tree being passed to change the base type
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(
      move(tree_ptr), FLAGS_dt);

  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_default_compliant_material(default_material);
  plant->set_contact_model_parameters(model_parameters);

  DrakeVisualizer& visualizer_publisher =
      *builder.template AddSystem<DrakeVisualizer>(plant->get_rigid_body_tree(),
                                                   &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  builder.Connect(plant->state_output_port(),
                  visualizer_publisher.get_input_port(0));

  map<string, int> position_map = tree.computePositionNameToIndexMap();
  vector<int> fixed_joints;

  VectorXd x0 = VectorXd::Zero(num_states);
  VectorXd x_sol = VectorXd::Zero(num_states);

  if (fixed_base) {
    x0(position_map.at("hip_roll_left")) = 0.1;
    x0(position_map.at("hip_roll_right")) = -0.1;
    x0(position_map.at("hip_yaw_left")) = 0.01;
    x0(position_map.at("hip_yaw_right")) = -0.01;
    x0(position_map.at("hip_pitch_left")) = .269;
    x0(position_map.at("hip_pitch_right")) = .269;
    x0(position_map.at("knee_left")) = -.744;
    x0(position_map.at("knee_right")) = -.744;
    x0(position_map.at("ankle_joint_left")) = .81;
    x0(position_map.at("ankle_joint_right")) = .81;
    x0(position_map.at("toe_left")) = 0;
    x0(position_map.at("toe_right")) = 0;
  } else {
    x0(position_map.at("base_z")) = 3;
    x0(position_map.at("hip_roll_left")) = 0.1;
    x0(position_map.at("hip_roll_right")) = -0.1;
    x0(position_map.at("hip_yaw_left")) = 0.01;
    x0(position_map.at("hip_yaw_right")) = -0.01;
    x0(position_map.at("hip_pitch_left")) = .269;
    x0(position_map.at("hip_pitch_right")) = .269;
    x0(position_map.at("knee_left")) = -.744;
    x0(position_map.at("knee_right")) = -.744;
    x0(position_map.at("ankle_joint_left")) = .81;
    x0(position_map.at("ankle_joint_right")) = .81;
    x0(position_map.at("toe_left")) = -60.0 * M_PI / 180.0;
    x0(position_map.at("toe_right")) = -60.0 * M_PI / 180.0;
    fixed_joints.push_back(position_map.at("base_roll"));
    fixed_joints.push_back(position_map.at("base_yaw"));
    // fixed_joints.push_back(position_map.at("hip_roll_left"));
    // fixed_joints.push_back(position_map.at("hip_roll_right"));
    // fixed_joints.push_back(position_map.at("hip_yaw_left"));
    // fixed_joints.push_back(position_map.at("hip_yaw_right"));
    fixed_joints.push_back(position_map.at("hip_pitch_left"));
    fixed_joints.push_back(position_map.at("hip_pitch_right"));
  }

  VectorXd q0 = x0.head(num_positions);
  VectorXd u0 = VectorXd::Zero(num_efforts);
  VectorXd lambda0;
  if (fixed_base) {
    lambda0 = VectorXd::Zero(num_position_constraints);
  } else {
    lambda0 = VectorXd::Zero(num_forces);
  }

  VectorXd q_sol, u_sol, lambda_sol;

  // Collison detect
  ContactInfo contact_info;
  if (!fixed_base) {
    // Contact information is specific to the floating base RBT
    VectorXd phi_total;
    Matrix3Xd normal_total, xA_total, xB_total;
    vector<int> idxA_total, idxB_total;
    KinematicsCache<double> k_cache =
        tree.doKinematics(x0.head(num_positions), x0.tail(num_velocities));

    tree.collisionDetect(k_cache, phi_total, normal_total, xA_total, xB_total,
                         idxA_total, idxB_total);

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

    contact_info = {xB, idxB};
  }

  // PositionSolver position_solver(tree);
  // position_solver.SetInitialGuessQ(q0);
  // position_solver.AddJointLimitConstraint(0.001);

  // cout << "Position solver result (Fixed base): " <<
  // position_solver.Solve(q0)
  //     << endl;

  // q_sol = position_solver.GetSolutionQ();

  // FixedPointSolver fp_solver(tree);
  // fp_solver.SetInitialGuess(q0, u0, lambda0);
  // fp_solver.AddJointLimitConstraint(0.001);

  // cout << "Fixed point solver result (Fixed base): " << fp_solver.Solve(q0,
  // u0)
  //     << endl;

  // q_sol = fp_solver.GetSolutionQ();
  // u_sol = fp_solver.GetSolutionU();
  // lambda_sol = fp_solver.GetSolutionLambda();

  // ContactSolver contact_solver(tree, contact_info);
  // contact_solver.SetInitialGuessQ(q0);
  // contact_solver.AddJointLimitConstraint(0.001);

  // std::cout << "Contact solver result (Floating base): "
  //          << contact_solver.Solve(q0) << std::endl;

  // q_sol = contact_solver.GetSolutionQ();

  FixedPointSolver fp_solver(tree, contact_info);
  fp_solver.SetInitialGuess(q0, u0, lambda0);
  fp_solver.AddSpreadNormalForcesCost();
  fp_solver.AddFrictionConeConstraint(0.8);
  fp_solver.AddJointLimitConstraint(0.001);

  cout << "Fixed point solver result (Floating base): "
       << fp_solver.Solve(q0, u0, fixed_joints) << endl;

  q_sol = fp_solver.GetSolutionQ();
  u_sol = fp_solver.GetSolutionU();
  lambda_sol = fp_solver.GetSolutionLambda();

  x_sol.head(num_positions) = q_sol;

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant,
                                          &simulator.get_mutable_context());

  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state =
        context.get_mutable_continuous_state();
    state.SetFromVector(x_sol);
  }

  context.FixInputPort(0, VectorXd::Zero(num_efforts));

  // simulator.set_publish_every_time_step(false);
  // simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  lcm.StartReceiveThread();

  simulator.StepTo(0.000000001);

  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
