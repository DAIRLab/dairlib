#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <gflags/gflags.h>

#include "attic/multibody/multibody_solvers.h"
#include "attic/multibody/rigidbody_utils.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "systems/goldilocks_models/file_utils.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::SolutionResult;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::trajectories::PiecewisePolynomial;

using dairlib::goldilocks_models::readCSV;
using dairlib::goldilocks_models::writeCSV;
using dairlib::multibody::ContactInfo;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;

DEFINE_string(init_file, "", "the file name of initial guess");
DEFINE_string(data_directory, "../dairlib_data/cassie_trajopt_data/",
              "directory to save/read data");
DEFINE_bool(store_data, false, "To store solution or not");
DEFINE_int32(max_iter, 100000, "Iteration limit");
DEFINE_double(duration, 0.4, "Duration of the single support phase (s)");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");

// Parameters which enable dircon-improving features
DEFINE_bool(is_scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(is_scale_variable, false, "Scale the decision variable");

namespace dairlib {

/// Trajectory optimization of fixed-spring cassie squatting
/// With the default initial guess, the solving time is about 2 mins.

// Constraint to fix the position of a point on a body (for initial guess)
class BodyPointPositionConstraint : public DirconAbstractConstraint<double> {
 public:
  BodyPointPositionConstraint(const RigidBodyTree<double>& tree,
                              string body_name, Vector3d translation,
                              Vector3d desired_pos)
      : DirconAbstractConstraint<double>(3, tree.get_num_positions(),
                                         VectorXd::Zero(3), VectorXd::Zero(3),
                                         body_name + "_position_constraint"),
        tree_(tree),
        body_idx_(multibody::GetBodyIndexFromName(tree, body_name)),
        translation_(translation),
        desired_pos_(desired_pos) {}
  ~BodyPointPositionConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;
    KinematicsCache<double> cache = tree_.doKinematics(q);
    VectorXd pt = tree_.transformPoints(cache, translation_, body_idx_, 0);

    *y = pt - desired_pos_;
  };

 private:
  const RigidBodyTree<double>& tree_;
  const int body_idx_;
  const Vector3d translation_;
  const Vector3d desired_pos_;
};

// Use fixed-point solver to get initial guess
void GetInitFixedPointGuess(const Vector3d& pelvis_position,
                            const RigidBodyTree<double>& tree, VectorXd* q_init,
                            VectorXd* u_init, VectorXd* lambda_init) {
  int n_q = tree.get_num_positions();
  int n_v = tree.get_num_velocities();
  int n_u = tree.get_num_actuators();

  int toe_left_idx = GetBodyIndexFromName(tree, "toe_left");
  int toe_right_idx = GetBodyIndexFromName(tree, "toe_right");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  MatrixXd xa(3, 4);
  xa.col(0) = pt_front_contact;
  xa.col(1) = pt_rear_contact;
  xa.col(2) = pt_front_contact;
  xa.col(3) = pt_rear_contact;
  std::vector<int> idxa;
  idxa.push_back(toe_left_idx);
  idxa.push_back(toe_left_idx);
  idxa.push_back(toe_right_idx);
  idxa.push_back(toe_right_idx);
  ContactInfo contact_info(xa, idxa);

  VectorXd q_desired = VectorXd::Zero(n_q);
  q_desired << 0, 0, 1.057, 1, 0, 0, 0, 0.0185, -0.0185, 0, 0, 0.383, 0.383,
      -1.02, -1.02, 1.24, 1.24, -1.48, -1.48;
  // The above numbers comes from one (FixedPointSolver) solution of cassie
  // standing

  std::map<int, double> fixed_joints;
  // floating base x, y z position
  for (int i = 0; i < 3; i++) {
    fixed_joints[i] = pelvis_position[i];
  }
  // floating base quaternion
  fixed_joints[3] = 1;
  fixed_joints[4] = 0;
  fixed_joints[5] = 0;
  fixed_joints[6] = 0;
  // hip yaw position
  // fixed_joints[9] = 0;
  // fixed_joints[10] = 0;

  FixedPointSolver fp_solver(tree, contact_info, q_desired, VectorXd::Zero(n_u),
                             MatrixXd::Zero(n_q, n_q),
                             MatrixXd::Identity(n_u, n_u));
  fp_solver.AddFrictionConeConstraint(0.8);
  fp_solver.AddJointLimitConstraint(0);  // 0.1
  fp_solver.AddFixedJointsConstraint(fixed_joints);
  fp_solver.AddSpreadNormalForcesCost();

  // get mathematicalprogram to add constraint ourselves
  shared_ptr<MathematicalProgram> mp = fp_solver.get_program();
  auto& q_var =
      mp->decision_variables().head(n_q);  // Assume q is located at the start
  Vector3d desired_left_toe_pos(0.06, 0.4, 0);
  Vector3d desired_right_toe_pos(0.06, -0.4, 0);
  auto left_foot_constraint = std::make_shared<BodyPointPositionConstraint>(
      tree, "toe_left", pt_front_contact, desired_left_toe_pos);
  auto right_foot_constraint = std::make_shared<BodyPointPositionConstraint>(
      tree, "toe_right", pt_front_contact, desired_right_toe_pos);
  mp->AddConstraint(left_foot_constraint, q_var);
  mp->AddConstraint(right_foot_constraint, q_var);

  VectorXd init_guess = VectorXd::Random(mp->decision_variables().size());
  // Provide initial guess to shorten the runtime
  // The numbers comes from one (FixedPointSolver) solution of cassie standing
  init_guess << 0, 0, 1.05263, 1, 0, 0, 0, 0.0185236, -0.0185236, 0, 0, 0.3836,
      0.3836, -1.026, -1.026, 1.249, 1.249, -1.480, -1.480, -0.1535, 0.1682,
      0.1407, -0.1843, -6.124, -5.841, 35.76, 35.79, -5.46, -5.439, -398.5,
      -396.3, 93.49, 17.75, 1.521, 68.57, -17.61, -1.503, 93.28, -0.7926,
      -1.762, 68.42, 0.652, 1.744;
  mp->SetInitialGuessForAllVariables(init_guess);

  //  mp->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
  //                      "../snopt.out");
  // target nonlinear constraint violation
  // mp->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                     "Major optimality tolerance", 1e-6);
  // target complementarity gap
  mp->SetSolverOption(drake::solvers::SnoptSolver::id(),
                      "Major feasibility tolerance", 1e-8);

  // solve for the standing pose
  cout << "Solving for fixed point...\n";
  const auto result = fp_solver.Solve();
  SolutionResult solution_result = result.get_solution_result();
  cout << to_string(solution_result) << endl;
  // cout << result.GetSolution() << endl;

  VectorXd q_sol = fp_solver.GetSolutionQ();
  VectorXd u_sol = fp_solver.GetSolutionU();
  VectorXd lambda_sol = fp_solver.GetSolutionLambda();

  VectorXd q_sol_reorder(n_q);
  q_sol_reorder << q_sol.segment(3, 4), q_sol.segment(0, 3), q_sol.tail(12);
  // Careful that the contact constraint ordering should be consistent with
  // those you set in DIRCON
  VectorXd lambda_sol_reorder(lambda_sol.size());
  VectorXd lambda_sol_contact = lambda_sol.tail(3 * idxa.size());
  for (unsigned int i = 0; i < idxa.size(); i++) {
    // We need to reorder cause contact toolkit's lambda ordering is different
    // from dircon order
    VectorXd lambda_dummy = lambda_sol_contact.segment(3 * i, 3);
    lambda_sol_contact(0 + 3 * i) = lambda_dummy(1);
    lambda_sol_contact(1 + 3 * i) = -lambda_dummy(2);
    lambda_sol_contact(2 + 3 * i) = lambda_dummy(0);
  }
  lambda_sol_reorder << lambda_sol_contact,
      lambda_sol.head(tree.getNumPositionConstraints());

  *q_init = q_sol_reorder;
  *u_init = u_sol;
  *lambda_init = lambda_sol_reorder;

  // Build temporary diagram for visualization
  VectorXd x(n_q + n_v);
  x << q_sol, VectorXd::Zero(n_v);
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  const PiecewisePolynomial<double> pp_xtraj = PiecewisePolynomial<double>(x);
  auto state_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto publisher =
      builder.AddSystem<drake::systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(0.2);
}

void DoMain(double duration, int max_iter, string data_directory,
            string init_file, double tol, bool to_store_data) {
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant, &scene_graph);

  string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.Finalize();

  // Create maps for joints
  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> velocities_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> actuators_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  // int n_x = n_q + n_v;

  // Set up contact/distance constraints
  const Body<double>& toe_left = plant.GetBodyByName("toe_left");
  const Body<double>& toe_right = plant.GetBodyByName("toe_right");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  bool isXZ = false;
  Vector3d ground_normal(0, 0, 1);
  auto left_toe_front_constraint = DirconPositionData<double>(
      plant, toe_left, pt_front_contact, isXZ, ground_normal);
  auto left_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_left, pt_rear_contact, isXZ, ground_normal);
  auto right_toe_front_constraint = DirconPositionData<double>(
      plant, toe_right, pt_front_contact, isXZ, ground_normal);
  auto right_toe_rear_constraint = DirconPositionData<double>(
      plant, toe_right, pt_rear_contact, isXZ, ground_normal);
  double mu = 1;
  left_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  left_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);

  const auto& thigh_left = plant.GetBodyByName("thigh_left");
  const auto& heel_spring_left = plant.GetBodyByName("heel_spring_left");
  const auto& thigh_right = plant.GetBodyByName("thigh_right");
  const auto& heel_spring_right = plant.GetBodyByName("heel_spring_right");
  double rod_length = 0.5012;  // from cassie_utils
  Vector3d pt_on_heel_spring = Vector3d(.11877, -.01, 0.0);
  Vector3d pt_on_thigh_left = Vector3d(0.0, 0.0, 0.045);
  Vector3d pt_on_thigh_right = Vector3d(0.0, 0.0, -0.045);
  auto distance_constraint_left = DirconDistanceData<double>(
      plant, thigh_left, pt_on_thigh_left, heel_spring_left, pt_on_heel_spring,
      rod_length);
  auto distance_constraint_right = DirconDistanceData<double>(
      plant, thigh_right, pt_on_thigh_right, heel_spring_right,
      pt_on_heel_spring, rod_length);

  // get rid of redundant constraint
  vector<int> skip_constraint_inds;
  skip_constraint_inds.push_back(3);
  skip_constraint_inds.push_back(9);

  // Double stance (all four contact points)
  vector<DirconKinematicData<double>*> double_stance_all_constraint;
  double_stance_all_constraint.push_back(&left_toe_front_constraint);
  double_stance_all_constraint.push_back(&left_toe_rear_constraint);
  double_stance_all_constraint.push_back(&right_toe_front_constraint);
  double_stance_all_constraint.push_back(&right_toe_rear_constraint);
  double_stance_all_constraint.push_back(&distance_constraint_left);
  double_stance_all_constraint.push_back(&distance_constraint_right);
  auto double_all_dataset = DirconKinematicDataSet<double>(
      plant, &double_stance_all_constraint, skip_constraint_inds);
  auto double_all_options =
      DirconOptions(double_all_dataset.countConstraints(), plant);
  // Be careful in setting relative constraint, because we skip constraints
  ///                 || lf    | lr    | rf    | rr      | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7 8 | 9 10 11 | 12 13
  /// After skipping  || 0 1 2 |   3 4 | 5 6 7 |   8  9  | 10 11
  double_all_options.setConstraintRelative(0, true);
  double_all_options.setConstraintRelative(1, true);
  double_all_options.setConstraintRelative(3, true);
  double_all_options.setConstraintRelative(5, true);
  double_all_options.setConstraintRelative(6, true);
  double_all_options.setConstraintRelative(8, true);
  // Constraint scaling
  if (FLAGS_is_scale_constraint) {
    // Dynamic constraints
    double s_dyn_1 = (FLAGS_is_scale_variable) ? 2.0 : 1.0;
    double s_dyn_2 = (FLAGS_is_scale_variable) ? 6.0 : 1.0;
    double s_dyn_3 = (FLAGS_is_scale_variable) ? 85.0 : 1.0;
    double_all_options.setDynConstraintScaling(
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}, 1.0 / 150.0);
    double_all_options.setDynConstraintScaling({15, 16},
                                               1.0 / 150.0 / 3.33 / s_dyn_1);
    double_all_options.setDynConstraintScaling({17, 18}, 1.0 / 150.0);
    double_all_options.setDynConstraintScaling({19, 20, 21, 22, 23, 24, 25, 26},
                                               1.0 / 150.0 / s_dyn_1);
    double_all_options.setDynConstraintScaling({27, 28}, 1.0 / 150.0 / s_dyn_2);
    double_all_options.setDynConstraintScaling({29, 30, 31, 32, 33, 34},
                                               1.0 / 150.0 / 10);
    double_all_options.setDynConstraintScaling({35, 36},
                                               1.0 / 150.0 / 15.0 / s_dyn_3);
    // Kinematic constraints
    int n_kin = double_all_dataset.countConstraints();
    double s_kin_vel = 500;
    double s_kin_acc = 1000;
    double s_kin_1 = (FLAGS_is_scale_variable) ? 10.0 : 1.0;
    double s_kin_2 = (FLAGS_is_scale_variable) ? 2.0 : 1.0;
    double_all_options.setKinConstraintScaling({0, 9}, 1.0 / 500.0 / s_kin_1);
    double_all_options.setKinConstraintScaling({10, 11}, 2.0 / 50.0 / s_kin_1);
    double_all_options.setKinConstraintScaling(
        {n_kin + 0, n_kin + 1, n_kin + 2, n_kin + 3, n_kin + 4, n_kin + 5,
         n_kin + 6, n_kin + 7, n_kin + 8, n_kin + 9},
        1.0 / 500.0 * s_kin_vel * s_kin_2 / s_kin_1);
    double_all_options.setKinConstraintScaling(
        {n_kin + 10, n_kin + 11}, 2.0 / 50.0 * s_kin_vel * s_kin_2 / s_kin_1);
    double_all_options.setKinConstraintScaling(
        {2 * n_kin + 0, 2 * n_kin + 1, 2 * n_kin + 2, 2 * n_kin + 3,
         2 * n_kin + 4, 2 * n_kin + 5, 2 * n_kin + 6, 2 * n_kin + 7,
         2 * n_kin + 8, 2 * n_kin + 9},
        1.0 / 500.0 * s_kin_acc * s_kin_2 / s_kin_1);
    double_all_options.setKinConstraintScaling(
        {2 * n_kin + 10, 2 * n_kin + 11},
        2.0 / 50.0 * s_kin_acc * s_kin_2 / s_kin_1);
  }

  // timesteps and modes setting
  vector<double> min_dt;
  vector<double> max_dt;
  min_dt.push_back(.01);
  max_dt.push_back(.3);
  vector<int> num_time_samples;
  num_time_samples.push_back(20);
  vector<DirconKinematicDataSet<double>*> dataset_list;
  vector<DirconOptions> options_list;
  dataset_list.push_back(&double_all_dataset);
  options_list.push_back(double_all_options);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // Snopt settings
  //   trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                            "Print file", "../snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", max_iter);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Iterations limit", 100000);  // QP subproblems
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Scale option",
                           0);  // snopt doc said try 2 if seeing snopta exit 40
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           tol);  // target nonlinear constraint violation
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major feasibility tolerance",
                           tol);  // target complementarity gap

  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // because of overlaps between modes

  // Get the decision variables that will be used
  auto u = trajopt->input();
  auto x = trajopt->state();
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);
  auto xmid = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] / 2);

  // height constraint
  trajopt->AddLinearConstraint(x0(positions_map.at("base_z")) == 1);
  // trajopt->AddLinearConstraint(xmid(positions_map.at("base_z")) == 1.1);
  trajopt->AddLinearConstraint(xf(positions_map.at("base_z")) == 1.1);

  // initial pelvis position
  // trajopt->AddLinearConstraint(x0(positions_map.at("base_y")) == 0);

  // pelvis pose constraints
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("base_qw")) == 1);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("base_qx")) == 0);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("base_qy")) == 0);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("base_qz")) == 0);

  // start/end velocity constraints
  trajopt->AddLinearConstraint(x0.tail(n_v) == VectorXd::Zero(n_v));
  trajopt->AddLinearConstraint(xf.tail(n_v) == VectorXd::Zero(n_v));

  // create joint/motor names
  vector<std::pair<string, string>> l_r_pairs{
      std::pair<string, string>("_left", "_right"),
      std::pair<string, string>("_right", "_left"),
  };
  vector<string> asy_joint_names{
      "hip_roll",
      "hip_yaw",
  };
  vector<string> sym_joint_names{"hip_pitch", "knee", "ankle_joint", "toe"};
  vector<string> joint_names{};
  vector<string> motor_names{};
  for (auto l_r_pair : l_r_pairs) {
    for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
      joint_names.push_back(asy_joint_names[i] + l_r_pair.first);
      motor_names.push_back(asy_joint_names[i] + l_r_pair.first + "_motor");
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      joint_names.push_back(sym_joint_names[i] + l_r_pair.first);
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        motor_names.push_back(sym_joint_names[i] + l_r_pair.first + "_motor");
      }
    }
  }

  // joint limits
  for (const auto& member : joint_names) {
    trajopt->AddConstraintToAllKnotPoints(
        x(positions_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt->AddConstraintToAllKnotPoints(
        x(positions_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }

  // u limit
  for (int i = 0; i < N; i++) {
    auto ui = trajopt->input(i);
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  // toe position constraint in y direction (avoid leg crossing)
  auto left_foot_constraint = std::make_shared<PointPositionConstraint<double>>(
      plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
      VectorXd::Ones(1) * 0.05,
      VectorXd::Ones(1) * std::numeric_limits<double>::infinity());
  auto right_foot_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          -std::numeric_limits<double>::infinity() * VectorXd::Ones(1),
          -0.05 * VectorXd::Ones(1));
  // scaling
  if (FLAGS_is_scale_constraint) {
    std::unordered_map<int, double> odbp_constraint_scale;
    odbp_constraint_scale.insert(std::pair<int, double>(0, 0.5));
    left_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
    right_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
  }
  for (int index = 0; index < num_time_samples[0]; index++) {
    auto x = trajopt->state(index);
    trajopt->AddConstraint(left_foot_constraint, x.head(n_q));
    trajopt->AddConstraint(right_foot_constraint, x.head(n_q));
  }

  // add cost
  const MatrixXd Q = 10 * 12.5 * MatrixXd::Identity(n_v, n_v);
  const MatrixXd R = 12.5 * MatrixXd::Identity(n_u, n_u);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  // Scale variable
  // Scaling decision variable doesn't seem to help in the task of squatting.
  // One hypothesis is that the initial guess we feed to the solver is very
  // good, so the variable scaling doesn't matter to much.
  if (FLAGS_is_scale_variable) {
    // time
    trajopt->ScaleTimeVariables(0.015);
    // state
    std::vector<int> idx_list;
    for (int i = n_q; i <= n_q + 9; i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleStateVariables(idx_list, 6);
    idx_list.clear();
    for (int i = n_q + 10; i <= n_q + n_v - 1; i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleStateVariables(idx_list, 3);
    // input
    trajopt->ScaleInputVariables({0, 1}, 60);
    trajopt->ScaleInputVariables({2, 3}, 300);  // 300
    trajopt->ScaleInputVariables({4, 7}, 60);
    trajopt->ScaleInputVariables({8, 9}, 600);  // 600
    // force
    trajopt->ScaleForceVariables(0, {0, 1}, 10);
    trajopt->ScaleForceVariables(0, {2, 2}, 1000);  // 1000
    trajopt->ScaleForceVariables(0, {3, 4}, 10);
    trajopt->ScaleForceVariable(0, 5, 1000);
    trajopt->ScaleForceVariables(0, {6, 7}, 10);
    trajopt->ScaleForceVariable(0, 8, 1000);
    trajopt->ScaleForceVariables(0, {9, 10}, 10);
    trajopt->ScaleForceVariable(0, 11, 1000);
    trajopt->ScaleForceVariables(0, {12, 13}, 600);

    // Print out the scaling factors
    /*for (int i=0; i < trajopt->decision_variables().size() ; i++) {
      cout << trajopt->decision_variable(i) << ", ";
      cout << trajopt->decision_variable(i).get_id() << ", ";
      cout << trajopt->FindDecisionVariableIndex(trajopt->decision_variable(i))
          << ", ";
      auto scale_map = trajopt->GetVariableScaling();
      auto it = scale_map.find(i);
      if (it != scale_map.end()) {
        cout << it->second;
      }
      cout << endl;
    }*/
  }

  // initial guess
  if (!init_file.empty()) {
    MatrixXd z0 = readCSV(data_directory + init_file);
    trajopt->SetInitialGuessForAllVariables(z0);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    trajopt->SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt->decision_variables().size()));

    // Use RBT fixed point solver for state/input/force
    RigidBodyTree<double> tree;
    buildCassieTree(tree, "examples/Cassie/urdf/cassie_fixed_springs.urdf",
                    drake::multibody::joints::kQuaternion, false);
    const double terrain_size = 100;
    const double terrain_depth = 0.20;
    drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);

    VectorXd q_init;
    VectorXd u_init;
    VectorXd lambda_init;
    VectorXd prev_lambda_init;

    for (int i = 0; i < N; i++) {
      Vector3d pelvis_position(0, 0, 1 + 0.1 * i / (N - 1));
      GetInitFixedPointGuess(pelvis_position, tree, &q_init, &u_init,
                             &lambda_init);

      // guess for state
      auto xi = trajopt->state(i);
      VectorXd xi_init(n_q + n_v);
      xi_init << q_init.head(4), q_init.tail(n_q - 4), VectorXd::Zero(n_v);
      trajopt->SetInitialGuess(xi, xi_init);

      // guess for input
      auto ui = trajopt->input(i);
      trajopt->SetInitialGuess(ui, u_init);

      // guess for constraint force
      auto lambdai = trajopt->force(0, i);
      trajopt->SetInitialGuess(lambdai, lambda_init);

      // guess for constraint force at collocation points
      // prev_lambda_init = lambda_init;
    }
  }
  // Careful: MUST set the initial guess for quaternion, since 0-norm quaternion
  // produces NAN value in some calculation.
  for (int i = 0; i < N; i++) {
    auto xi = trajopt->state(i);
    if ((trajopt->GetInitialGuess(xi.head(4)).norm() == 0) ||
        std::isnan(trajopt->GetInitialGuess(xi.head(4)).norm())) {
      trajopt->SetInitialGuess(xi(0), 1);
      trajopt->SetInitialGuess(xi(1), 0);
      trajopt->SetInitialGuess(xi(2), 0);
      trajopt->SetInitialGuess(xi(3), 0);
    }
  }

  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(*trajopt).name() << endl;

  cout << "Solving DIRCON\n\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  SolutionResult solution_result = result.get_solution_result();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  // trajopt->PrintSolution();
  for (int i = 0; i < 100; i++) {
    cout << '\a';
  }  // making noise to notify
  cout << "\n" << to_string(solution_result) << endl;
  cout << "Solve time:" << elapsed.count() << std::endl;
  cout << "Cost:" << result.get_optimal_cost() << std::endl;

  // Check which solver was used
  cout << "Solver: " << result.get_solver_id().name() << endl;

  // Check if the nonlinear constraints are all satisfied
  // bool constraint_satisfied = solvers::CheckGenericConstraints(*trajopt,
  //                             result, tol);
  // cout << "constraint_satisfied = " << constraint_satisfied << endl;

  // store the solution of the decision variable
  VectorXd z = result.GetSolution(trajopt->decision_variables());
  if (to_store_data) {
    writeCSV(data_directory + string("z.csv"), z);
  }
  // for (int i = 0; i < z.size(); i++) {
  //   cout << trajopt->decision_variables()[i] << ", " << z[i] << endl;
  // }
  // cout << endl;

  // store the time, state, and input at knot points
  VectorXd time_at_knots = trajopt->GetSampleTimes(result);
  MatrixXd state_at_knots = trajopt->GetStateSamples(result);
  MatrixXd input_at_knots = trajopt->GetInputSamples(result);
  state_at_knots.col(N - 1) = result.GetSolution(xf);
  cout << "time_at_knots = \n" << time_at_knots << "\n";
  cout << "state_at_knots = \n" << state_at_knots << "\n";
  cout << "state_at_knots.size() = " << state_at_knots.size() << endl;
  cout << "input_at_knots = \n" << input_at_knots << "\n";
  if (to_store_data) {
    writeCSV(data_directory + string("t_i.csv"), time_at_knots);
    writeCSV(data_directory + string("x_i.csv"), state_at_knots);
    writeCSV(data_directory + string("u_i.csv"), input_at_knots);
  }

  // Store lambda
  std::ofstream ofile;
  ofile.open(data_directory + "lambda.txt", std::ofstream::out);
  cout << "lambda_sol = \n";
  for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
    for (int index = 0; index < num_time_samples[mode]; index++) {
      auto lambdai = trajopt->force(mode, index);
      cout << result.GetSolution(lambdai).transpose() << endl;
      ofile << result.GetSolution(lambdai).transpose() << endl;
    }
  }
  ofile.close();

  // visualizer
  const PiecewisePolynomial<double> pp_xtraj =
      trajopt->ReconstructStateTrajectory(result);

  auto traj_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      plant.num_positions() + plant.num_velocities(), 0, plant.num_positions());
  builder.Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose =
      builder.AddSystem<MultibodyPositionToGeometryPose<double>>(plant);
  builder.Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  // *******Add COM visualization**********
  bool plot_com = true;
  bool com_on_ground = true;
  auto ball_plant = std::make_unique<MultibodyPlant<double>>();
  if (plot_com) {
    double radius = .02;
    UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
    SpatialInertia<double> M_Bcm(1, Eigen::Vector3d::Zero(), G_Bcm);

    const drake::multibody::RigidBody<double>& ball =
        ball_plant->AddRigidBody("Ball", M_Bcm);

    ball_plant->RegisterAsSourceForSceneGraph(&scene_graph);
    // Add visual for the COM.
    const Eigen::Vector4d orange(1.0, 0.55, 0.0, 1.0);
    const RigidTransformd X_BS = RigidTransformd::Identity();
    ball_plant->RegisterVisualGeometry(ball, X_BS, Sphere(radius), "visual",
                                       orange);
    ball_plant->Finalize();

    // connect
    auto q_passthrough = builder.AddSystem<SubvectorPassThrough>(
        plant.num_positions() + plant.num_velocities(), 0,
        plant.num_positions());
    builder.Connect(traj_source->get_output_port(),
                    q_passthrough->get_input_port());
    auto rbt_passthrough = builder.AddSystem<multibody::ComPoseSystem>(plant);

    auto ball_to_pose =
        builder.AddSystem<MultibodyPositionToGeometryPose<double>>(*ball_plant);
    builder.Connect(*q_passthrough, *rbt_passthrough);
    if (com_on_ground) {
      builder.Connect(rbt_passthrough->get_xy_com_output_port(),
                      ball_to_pose->get_input_port());
    } else {
      builder.Connect(rbt_passthrough->get_com_output_port(),
                      ball_to_pose->get_input_port());
    }
    builder.Connect(
        ball_to_pose->get_output_port(),
        scene_graph.get_source_pose_port(ball_plant->get_source_id().value()));
  }
  // **************************************

  drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }

  return;
}
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain(FLAGS_duration, FLAGS_max_iter, FLAGS_data_directory,
                  FLAGS_init_file, FLAGS_tol, FLAGS_store_data);
}
