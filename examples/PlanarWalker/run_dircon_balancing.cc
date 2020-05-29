#include <chrono>

#include <gflags/gflags.h>

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

#include "attic/multibody/multibody_solvers.h"
#include "attic/multibody/utility_systems.h"

#include "common/find_resource.h"
#include "attic/systems/trajectory_optimization/dircon_util.h"
#include "attic/systems/trajectory_optimization/dircon_position_data.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "attic/systems/trajectory_optimization/hybrid_dircon.h"
#include "attic/systems/trajectory_optimization/dircon_options.h"
#include "attic/systems/trajectory_optimization/dircon_opt_constraints.h"

using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;
using std::map;
using std::unique_ptr;
using std::make_unique;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;

using drake::trajectories::PiecewisePolynomial;

using dairlib::multibody::ContactInfo;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::PositionSolver;
using drake::multibody::AddFlatTerrainToWorld;
using drake::solvers::MathematicalProgramResult;

using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;

DEFINE_double(duration, 1.0, "The total time for balancing");

DEFINE_double(init_height, 0.855, "Initial height of the pelvis");
DEFINE_double(planar_roty, 0.0, "Initial torso angle");
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

namespace dairlib {

class CoMHeightConstraint : public DirconAbstractConstraint<double> {
 public:
  CoMHeightConstraint(const RigidBodyTree<double>& tree, const VectorXd& lb,
                      const VectorXd& ub, std::string constraint_name,
                      double desired_height)
      : DirconAbstractConstraint<double>(1, tree.get_num_positions(), lb, ub,
                                         constraint_name),
        tree_(tree),
        desired_height_(desired_height) {}

  ~CoMHeightConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;
    KinematicsCache<double> cache = tree_.doKinematics(q);
    Vector3d pt = tree_.centerOfMass(cache);

    VectorXd y_sol = VectorXd::Zero(1);
    y_sol(0) = pt(2) - desired_height_;
    *y = y_sol;
  }

 private:
  const RigidBodyTree<double>& tree_;
  double desired_height_;
};

class BalancingConstraint : public DirconAbstractConstraint<double> {
 public:
  BalancingConstraint(const RigidBodyTree<double>& tree, std::string body_name)
      : DirconAbstractConstraint<double>(
            1, tree.get_num_positions(), VectorXd::Ones(1) * -0.01,
            VectorXd::Ones(1) * 0.01, "balancing_constraint"),
        tree_(tree),
        body_name_(body_name) {}

  ~BalancingConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;
    KinematicsCache<double> cache = tree_.doKinematics(q);
    Vector3d com_pos = tree_.centerOfMass(cache);
    int stance_idx = tree_.FindBodyIndex(body_name_);
    Vector3d foot_position =
        tree_.transformPoints(cache, Vector3d::Zero(), stance_idx, 0);

    VectorXd constraint = VectorXd::Zero(1);
    constraint << com_pos(0) - foot_position(0);
    *y = constraint;
  }

 private:
  const RigidBodyTree<double>& tree_;
  std::string body_name_;
  std::map<std::string, int> name_to_position_map_;
};

shared_ptr<HybridDircon<double>> runDircon(
    int argc, char* argv[], RigidBodyTree<double>& tree,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_lambda_traj,
    vector<PiecewisePolynomial<double>> init_lambda_slack_traj,
    vector<PiecewisePolynomial<double>> init_gamma_slack_traj) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  cout << "Bodies: " << endl;
  for (int i = 0; i < tree.get_num_bodies(); i++)
    cout << tree.getBodyOrFrameName(i) << endl;
  cout << "Actuators: " << endl;
  for (int i = 0; i < tree.get_num_actuators(); i++)
    cout << tree.actuators[i].name_ << endl;
  cout << "Positions: " << endl;
  for (int i = 0; i < tree.get_num_positions(); i++)
    cout << tree.get_position_name(i) << endl;
  cout << "Velocities: " << endl;
  for (int i = 0; i < tree.get_num_velocities(); i++)
    cout << tree.get_velocity_name(i) << endl;

  // Bodies:
  // world
  // base
  // base_x
  // base_xz
  // torso
  // torso_mass
  // left_upper_leg
  // left_upper_leg_mass
  // left_lower_leg
  // left_lower_leg_mass
  // left_foot
  // left_foot_mass
  // right_upper_leg
  // right_upper_leg_mass
  // right_lower_leg
  // right_lower_leg_mass
  // right_foot
  // right_foot_mass

  // Actuators:
  // left_hip_torque
  // right_hip_torque
  // left_knee_torque
  // left_ankle_torque
  // right_knee_torque
  // right_ankle_torque

  int left_foot_idx = tree.FindBodyIndex("left_foot");
  int right_foot_idx = tree.FindBodyIndex("right_foot");
  cout << left_foot_idx << " " << right_foot_idx << endl;

  Vector3d front_pt;
  front_pt << 0, 0, 0.075;
  Vector3d rear_pt;
  rear_pt << 0, 0, -0.075;
  bool isXZ = true;

  auto left_forward_constraint =
      DirconPositionData<double>(tree, left_foot_idx, front_pt, isXZ);
  auto left_rear_constraint =
      DirconPositionData<double>(tree, left_foot_idx, rear_pt, isXZ);
  auto right_forward_constraint =
      DirconPositionData<double>(tree, right_foot_idx, front_pt, isXZ);
  auto right_rear_constraint =
      DirconPositionData<double>(tree, right_foot_idx, rear_pt, isXZ);

  Vector3d normal;
  normal << 0, 0, 1;
  double mu = 0.8;
  left_forward_constraint.addFixedNormalFrictionConstraints(normal, mu);
  left_rear_constraint.addFixedNormalFrictionConstraints(normal, mu);
  right_forward_constraint.addFixedNormalFrictionConstraints(normal, mu);
  right_rear_constraint.addFixedNormalFrictionConstraints(normal, mu);

  vector<DirconKinematicData<double>*> left_foot_constraints;
  left_foot_constraints.push_back(&left_forward_constraint);
  left_foot_constraints.push_back(&left_rear_constraint);
  auto left_data_set =
      DirconKinematicDataSet<double>(tree, &left_foot_constraints);

  vector<DirconKinematicData<double>*> right_foot_constraints;
  right_foot_constraints.push_back(&right_forward_constraint);
  right_foot_constraints.push_back(&right_rear_constraint);
  auto right_data_set =
      DirconKinematicDataSet<double>(tree, &right_foot_constraints);

  auto left_options = DirconOptions(left_data_set.countConstraints());
  std::cout << "Left count constraints: " << left_data_set.countConstraints()
            << endl;
  // left_options.setConstraintRelative(0, true);
  left_options.setAllConstraintsRelative(true);

  auto right_options = DirconOptions(right_data_set.countConstraints());
  std::cout << "Right count constraints: " << right_data_set.countConstraints()
            << endl;
  // right_options.setConstraintRelative(0, true);
  right_options.setAllConstraintsRelative(true);

  std::vector<int> timesteps;
  timesteps.push_back(10);
  timesteps.push_back(10);
  std::vector<double> min_dt;
  min_dt.push_back(0.07);
  min_dt.push_back(0.07);
  std::vector<double> max_dt;
  max_dt.push_back(1.0);
  max_dt.push_back(1.0);

  vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&left_data_set);
  dataset_list.push_back(&right_data_set);

  vector<DirconOptions> options_list;
  options_list.push_back(left_options);
  options_list.push_back(right_options);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      tree, timesteps, min_dt, max_dt, dataset_list, options_list);

  trajopt->AddDurationBounds(FLAGS_duration, FLAGS_duration);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 500);

  for (uint j = 0; j < timesteps.size(); j++) {
    trajopt->drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt->SetInitialForceTrajectory(j, init_lambda_traj[j],
                                       init_lambda_slack_traj[j],
                                       init_gamma_slack_traj[j]);
  }

  // State
  // planar_x - 0, 9
  // planar_z - 1, 10
  // planar_roty - 2, 11
  // left_hip_pin - 3, 12
  // left_knee_pin - 4, 13
  // left_ankle - 5, 14
  // right_hip_pin - 6, 15
  // right_knee_pin - 7, 16
  // right_ankle - 8, 17
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->final_state();

  auto balancing_constraint =
      std::make_shared<BalancingConstraint>(tree, "right_foot");
  trajopt->AddConstraint(balancing_constraint,
                         xf.head(tree.get_num_positions()));

  // Initial pose position and velocity constraint
  trajopt->AddLinearConstraint(x0(0) == 0);
  trajopt->AddLinearConstraint(x0(1) == 0.80);
  trajopt->AddLinearConstraint(x0(2) == FLAGS_planar_roty);
  trajopt->AddLinearConstraint(x0(3) == FLAGS_left_hip_pin);
  trajopt->AddLinearConstraint(x0(4) == FLAGS_left_knee_pin);
  trajopt->AddLinearConstraint(x0(5) == FLAGS_left_ankle);
  trajopt->AddLinearConstraint(x0(6) == FLAGS_right_hip_pin);
  trajopt->AddLinearConstraint(x0(7) == FLAGS_right_knee_pin);
  trajopt->AddLinearConstraint(x0(8) == FLAGS_right_ankle);
  trajopt->AddLinearConstraint(x0(9) <= 0.1);
  trajopt->AddLinearConstraint(x0(10) <= 0.1);
  trajopt->AddLinearConstraint(x0(11) <= 0.1);
  trajopt->AddLinearConstraint(x0(12) <= 0.1);
  trajopt->AddLinearConstraint(x0(13) <= 0.1);
  trajopt->AddLinearConstraint(x0(14) <= 0.1);
  trajopt->AddLinearConstraint(x0(15) <= 0.1);
  trajopt->AddLinearConstraint(x0(16) <= 0.1);
  trajopt->AddLinearConstraint(x0(17) <= 0.1);
  trajopt->AddLinearConstraint(x0(9) >= -0.1);
  trajopt->AddLinearConstraint(x0(10) >= -0.1);
  trajopt->AddLinearConstraint(x0(11) >= -0.1);
  trajopt->AddLinearConstraint(x0(12) >= -0.1);
  trajopt->AddLinearConstraint(x0(13) >= -0.1);
  trajopt->AddLinearConstraint(x0(14) >= -0.1);
  trajopt->AddLinearConstraint(x0(15) >= -0.1);
  trajopt->AddLinearConstraint(x0(16) >= -0.1);
  trajopt->AddLinearConstraint(x0(17) >= -0.1);

  // Final pose velocity constraints
  trajopt->AddLinearConstraint(xf(9) <= 0.1);
  trajopt->AddLinearConstraint(xf(10) <= 0.1);
  trajopt->AddLinearConstraint(xf(11) <= 0.1);
  trajopt->AddLinearConstraint(xf(12) <= 0.1);
  trajopt->AddLinearConstraint(xf(13) <= 0.1);
  trajopt->AddLinearConstraint(xf(14) <= 0.1);
  trajopt->AddLinearConstraint(xf(15) <= 0.1);
  trajopt->AddLinearConstraint(xf(16) <= 0.1);
  trajopt->AddLinearConstraint(xf(17) <= 0.1);
  trajopt->AddLinearConstraint(xf(9) >= -0.1);
  trajopt->AddLinearConstraint(xf(10) >= -0.1);
  trajopt->AddLinearConstraint(xf(11) >= -0.1);
  trajopt->AddLinearConstraint(xf(12) >= -0.1);
  trajopt->AddLinearConstraint(xf(13) >= -0.1);
  trajopt->AddLinearConstraint(xf(14) >= -0.1);
  trajopt->AddLinearConstraint(xf(15) >= -0.1);
  trajopt->AddLinearConstraint(xf(16) >= -0.1);
  trajopt->AddLinearConstraint(xf(17) >= -0.1);

  VectorXd lower_bound = VectorXd::Ones(1) * -0.6;
  VectorXd upper_bound = VectorXd::Ones(1) * 0.2;
  std::string constraint_name = "com_height_constraint";
  auto com_constraint = std::make_shared<CoMHeightConstraint>(
      tree, lower_bound, upper_bound, constraint_name, 0.75);

  for(int i=0; i < 2; i++) {
    for(int j=0; j < 10; j++) {
      auto x = trajopt->state_vars_by_mode(i, j);
      trajopt->AddConstraint(com_constraint, x.head(tree.get_num_positions()));
    }
  }

  auto x = trajopt->state();
  // trajopt->AddConstraintToAllKnotPoints(x(1) >= 0.8);
  // trajopt->AddConstraintToAllKnotPoints(x(4) >= 0);
  // trajopt->AddConstraintToAllKnotPoints(x(7) >= 0);

  const double R = 1;  // Cost on input effort
  auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R*u);
  const double Q = 0;
  trajopt->AddRunningCost(x.transpose()*Q*x);


  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

  // systems::trajectory_optimization::checkConstraints(trajopt.get(), result);

  std::cout << "Result: " << result.get_solution_result() << std::endl;

  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt->ReconstructStateTrajectory(result);
  const drake::trajectories::PiecewisePolynomial<double> pp_utraj =
      trajopt->ReconstructInputTrajectory(result);
  // cout << pp_utraj.value(0.1).transpose() << endl;
  // cout << pp_utraj.value(0.5).transpose() << endl;
  // cout << pp_utraj.value(1.0).transpose() << endl;
  auto state_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto publisher =
      builder.AddSystem<drake::systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.2);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
  return trajopt;
}
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));

  RigidBodyTree<double> tree;
  std::string full_name = dairlib::FindResourceOrThrow(
      "examples/PlanarWalker/PlanarWalkerWithTorsoAndFeet.urdf");
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(full_name,
      drake::multibody::joints::kFixed, &tree);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(tree.get_num_positions() +
                                             tree.get_num_velocities());

  // std::map<std::string, int> map =
  //     tree.computePositionNameToIndexMap();
  // x0(map.at("planar_z")) = FLAGS_init_height;
  // x0(map.at("planar_roty")) = FLAGS_planar_roty;
  // x0(map.at("left_ankle")) = FLAGS_left_ankle;
  // x0(map.at("left_knee_pin")) = FLAGS_left_knee_pin;
  // x0(map.at("left_hip_pin")) = FLAGS_left_hip_pin;
  // x0(map.at("right_ankle")) = FLAGS_right_ankle;
  // x0(map.at("right_knee_pin")) = FLAGS_right_knee_pin;
  // x0(map.at("right_hip_pin")) = FLAGS_right_hip_pin;

  std::map<std::string, int> map = tree.computePositionNameToIndexMap();
  x0(map.at("planar_x")) = 0.0;
  x0(map.at("planar_z")) = 0.75;
  x0(map.at("planar_roty")) = 0.0;
  // x0(map.at("left_ankle")) = 0.58234;
  // x0(map.at("left_knee_pin")) = -1.16473;
  // x0(map.at("left_hip_pin")) = 0.58234;
  // x0(map.at("right_ankle")) = 0.58234;
  // x0(map.at("right_knee_pin")) = -1.16473;
  // x0(map.at("right_hip_pin")) = 0.58234;

  std::vector<int> fixed_joints;

  Eigen::VectorXd q0 = x0.head(tree.get_num_positions());
  PositionSolver position_solver(tree, q0);
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
  std::cout << q0 << std::endl;

  const double terrain_size = 100;
  const double terrain_depth = 1.0;
  drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);

  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;
  KinematicsCache<double> k_cache = tree.doKinematics(q0);

  // The full collisionDetect solution.
  tree.collisionDetect(k_cache, phi_total, normal_total, xA_total, xB_total,
                        idxA_total, idxB_total);

  const int world_ind =
      dairlib::multibody::GetBodyIndexFromName(tree, "world");
  const int toe_left_ind =
      dairlib::multibody::GetBodyIndexFromName(tree, "left_foot");
  const int toe_right_ind =
      dairlib::multibody::GetBodyIndexFromName(tree, "right_foot");

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

  unique_ptr<FixedPointSolver> fp_solver;
  int num_forces = tree.getNumPositionConstraints();
  num_forces += 3 * contact_info.num_contacts;
  std::cout << num_forces << std::endl;
  VectorXd lambda0 = VectorXd::Zero(num_forces);

  VectorXd u0 = VectorXd::Zero(tree.get_num_actuators());

  fp_solver = make_unique<FixedPointSolver>(tree, contact_info, q0, u0);
  fp_solver->SetInitialGuess(q0, u0, lambda0);
  fp_solver->AddSpreadNormalForcesCost();
  fp_solver->AddFrictionConeConstraint(0.8);
  fp_solver->AddFixedJointsConstraint(fixed_joints_map);
  fp_solver->AddJointLimitConstraint(0.001);

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

  x0.head(tree.get_num_positions()) << q;
  std::cout << x0.transpose() << std::endl;

  Eigen::VectorXd init_lambda_vec(2);
  init_lambda_vec << 0, tree.getMass() * 9.81;

  int nu = tree.get_num_actuators();
  int num_segments = 2;
  int num_knot_points = 10;

  vector<MatrixXd> init_x;
  vector<MatrixXd> init_u;
  vector<PiecewisePolynomial<double>> init_lambda_traj;
  vector<PiecewisePolynomial<double>> init_lambda_slack_traj;
  vector<PiecewisePolynomial<double>> init_gamma_slack_traj;

  vector<double> init_times;
  for(int i=0; i<num_segments*num_knot_points-1; i++) {
    init_times.push_back(i*0.2);
    init_x.push_back(x0);
    init_u.push_back(VectorXd::Random(nu));
  }
  auto init_x_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(init_times, init_x);
  auto init_u_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(init_times, init_u);

  for(int i=0; i<num_segments; i++) {
    vector<MatrixXd> init_lambda_i;
    vector<MatrixXd> init_lambda_slack_i;
    vector<MatrixXd> init_gamma_slack_i;
    vector<double> init_times_i;
    for(int i=0; i<num_segments; i++) {
      init_times_i.push_back(i*0.2);
      init_lambda_i.push_back(init_lambda_vec);
      init_lambda_slack_i.push_back(init_lambda_vec);
      init_gamma_slack_i.push_back(Vector2d::Zero());
    }

    auto init_lambda_traj_i =
        PiecewisePolynomial<double>::ZeroOrderHold(init_times_i, init_lambda_i);
    auto init_lambda_slack_traj_i = PiecewisePolynomial<double>::ZeroOrderHold(
        init_times_i, init_lambda_slack_i);
    auto init_gamma_slack_traj_i = PiecewisePolynomial<double>::ZeroOrderHold(
        init_times_i, init_gamma_slack_i);

    init_lambda_traj.push_back(init_lambda_traj_i);
    init_lambda_slack_traj.push_back(init_lambda_slack_traj_i);
    init_gamma_slack_traj.push_back(init_gamma_slack_traj_i);
  }

  auto prog = dairlib::runDircon(argc, argv, tree, init_x_traj, init_u_traj,
                                 init_lambda_traj, init_lambda_slack_traj,
                                 init_gamma_slack_traj);

  return 0;
}
