#include <chrono>
#include <memory>
#include <string>

#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/choose_best_solver.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization ::PointPositionConstraint;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

DEFINE_int32(knot_points, 10, "Number of knot points per contact mode");
DEFINE_double(stride_length, 0.2, "Stride length.");
DEFINE_double(start_height, 0.9, "Starting pelvis height for the trajectory.");
DEFINE_double(duration, 0.0, "Duration of the total gait");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");
DEFINE_double(tol, 1e-6, "Tolerance for constraint violation and dual gap");
DEFINE_string(load_filename, "", "File to load decision vars from.");
DEFINE_string(
    data_directory,
    "/home/yangwill/workspace/dairlib/examples/Cassie/saved_trajectories/",
    "Directory path to save decision vars to.");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision "
              "vars to.");
DEFINE_string(traj_name, "", "File to load saved LCM trajs from.");

namespace dairlib {

HybridDircon<double>* createDircon(MultibodyPlant<double>& plant);

void setKinematicConstraints(HybridDircon<double>* trajopt,
                             const MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForQStance(int num_knot_points,
                                        const MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForQFlight(int num_knot_points, double apex_height,
                                        const MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForV(const vector<VectorXd>& q_guess, double dt,
                                  const MultibodyPlant<double>& plant);
MatrixXd loadSavedDecisionVars(const string& filepath);

MatrixXd generateStateAndInputMatrix(const PiecewisePolynomial<double>& states,
                                     const PiecewisePolynomial<double>& inputs,
                                     VectorXd times);
vector<string> createStateNameVectorFromMap(const map<string, int>& pos_map,
                                            const map<string, int>& vel_map,
                                            const map<string, int>& act_map);
void printConstraint(const shared_ptr<HybridDircon<double>>& trajopt,
                     const MathematicalProgramResult& result);

void DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(0.0);
  //  Parser parser(&plant, &scene_graph);

  string file_name = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  //  if (FLAGS_use_springs) file_name = "examples/Cassie/urdf/cassie_v2.urdf";
  //  string full_name =
  //      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  //  parser.AddModelFromFile(full_name);
  //  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
  //                                                   Eigen::Vector3d::UnitZ());
  addCassieMultibody(&plant, &scene_graph, true, file_name, false, false);
  plant.Finalize();
  //  plant.Finalize();

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;

  std::cout << "nq: " << n_q << " n_v: " << n_v << " n_x: " << n_x << std::endl;
  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  // Set up contact/distance constraints
  const auto left_toe_front = LeftToeFront(plant);
  const auto right_toe_front = RightToeFront(plant);
  const auto left_toe_rear = LeftToeRear(plant);
  const auto right_toe_rear = RightToeRear(plant);

  bool isXZ = false;
  Vector3d ground_normal(0, 0, 1);
  auto left_toe_front_constraint =
      DirconPositionData<double>(plant, left_toe_front.second.body(),
                                 left_toe_front.first, isXZ, ground_normal);
  auto left_toe_rear_constraint =
      DirconPositionData<double>(plant, left_toe_rear.second.body(),
                                 left_toe_rear.first, isXZ, ground_normal);
  auto right_toe_front_constraint =
      DirconPositionData<double>(plant, right_toe_front.second.body(),
                                 right_toe_front.first, isXZ, ground_normal);
  auto right_toe_rear_constraint =
      DirconPositionData<double>(plant, right_toe_rear.second.body(),
                                 right_toe_rear.first, isXZ, ground_normal);
  double mu = 1;
  left_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  left_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_front_constraint.addFixedNormalFrictionConstraints(mu);
  right_toe_rear_constraint.addFixedNormalFrictionConstraints(mu);

  const auto& thigh_left = LeftRodOnThigh(plant);
  const auto& heel_spring_left = LeftRodOnHeel(plant);
  const auto& thigh_right = RightRodOnThigh(plant);
  const auto& heel_spring_right = RightRodOnHeel(plant);
  double rod_length = 0.5012;  // from cassie_utils
  auto distance_constraint_left = DirconDistanceData<double>(
      plant, thigh_left.second.body(), thigh_left.first,
      heel_spring_left.second.body(), heel_spring_left.first, rod_length);
  auto distance_constraint_right = DirconDistanceData<double>(
      plant, thigh_right.second.body(), thigh_right.first,
      heel_spring_right.second.body(), heel_spring_right.first, rod_length);

  // get rid of redundant constraint
  vector<int> skip_constraint_inds;
  skip_constraint_inds.push_back(3);

  // left stance (left leg constraint + loop closure constraints)
  vector<DirconKinematicData<double>*> left_stance_constraints;
  left_stance_constraints.push_back(&left_toe_front_constraint);
  left_stance_constraints.push_back(&left_toe_rear_constraint);
  left_stance_constraints.push_back(&distance_constraint_left);
  left_stance_constraints.push_back(&distance_constraint_right);

  // right stance (left leg constraint + loop closure constraints)
  vector<DirconKinematicData<double>*> right_stance_constraints;
  right_stance_constraints.push_back(&right_toe_front_constraint);
  right_stance_constraints.push_back(&right_toe_rear_constraint);
  right_stance_constraints.push_back(&distance_constraint_left);
  right_stance_constraints.push_back(&distance_constraint_right);

  auto left_stance_dataset = DirconKinematicDataSet<double>(
      plant, &left_stance_constraints, skip_constraint_inds);
  auto right_stance_dataset = DirconKinematicDataSet<double>(
      plant, &right_stance_constraints, skip_constraint_inds);

  auto left_stance_options =
      DirconOptions(left_stance_dataset.countConstraints(), plant);
  auto right_stance_options =
      DirconOptions(right_stance_dataset.countConstraints(), plant);
  // Set up options
  /// Constraint indices
  ///                 || lf/rf | lr/rr | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7
  /// After skipping  || 0 1 2 |   3 4 | 5 6
  /// Set all the non-z toe constraints to be relative
  left_stance_options.setConstraintRelative(0, true);
  left_stance_options.setConstraintRelative(1, true);
  left_stance_options.setConstraintRelative(3, true);
  right_stance_options.setConstraintRelative(0, true);
  right_stance_options.setConstraintRelative(1, true);
  right_stance_options.setConstraintRelative(3, true);

  // timesteps and modes setting
  vector<int> timesteps;  // Number of timesteps per mode
  timesteps.push_back(FLAGS_knot_points);
  timesteps.push_back(FLAGS_knot_points);
  vector<double> min_dt;  // min/max duration per timestep
  vector<double> max_dt;  // min_duration = knot_points * 0.01 ~= .1s
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  // Add contact modes and contact decision
  // variables to single vector for DIRCON
  std::vector<DirconKinematicDataSet<double>*> contact_mode_list;
  std::vector<DirconOptions> options_list;
  contact_mode_list.push_back(&left_stance_dataset);
  contact_mode_list.push_back(&right_stance_dataset);

  options_list.push_back(left_stance_options);
  options_list.push_back(right_stance_options);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, timesteps, min_dt, max_dt, contact_mode_list, options_list);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../walking_snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 50000);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Iterations limit", 50000);  // QP subproblems
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0
  trajopt->SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Scale option",
      FLAGS_scale_option);  // snopt doc said try 2 if seeing snopta exit 40
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           FLAGS_tol);  // target nonlinear constraint violation
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major feasibility tolerance",
                           FLAGS_tol);  // target complementarity gap

  std::cout << "Adding kinematic constraints: " << std::endl;
  setKinematicConstraints(trajopt.get(), plant);
  std::cout << "Setting initial conditions: " << std::endl;

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    MatrixXd decisionVars =
        loadSavedDecisionVars(FLAGS_data_directory + FLAGS_load_filename);
    trajopt->SetInitialGuessForAllVariables(decisionVars);
  } else {
    trajopt->SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt->decision_variables().size()));
  }

  // To avoid NaN quaternions
  for (int i = 0; i < trajopt->N(); i++) {
    auto xi = trajopt->state(i);
    if ((trajopt->GetInitialGuess(xi.head(4)).norm() == 0) ||
        std::isnan(trajopt->GetInitialGuess(xi.head(4)).norm())) {
      trajopt->SetInitialGuess(xi(0), 1);
      trajopt->SetInitialGuess(xi(1), 0);
      trajopt->SetInitialGuess(xi(2), 0);
      trajopt->SetInitialGuess(xi(3), 0);
    }
  }

  double alpha = .2;
  int num_poses = std::min(FLAGS_knot_points, 5);
  trajopt->CreateVisualizationCallback(file_name, num_poses, alpha);

  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(*trajopt).name() << endl;

  cout << "Solving DIRCON\n\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  //  SolutionResult solution_result = result.get_solution_result();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "Solve result: " << result.get_solution_result() << std::endl;

  // Save trajectory to file
  DirconTrajectory saved_traj(plant, *trajopt, result, "walking_trajectory",
                              "Decision variables and state/input trajectories "
                              "for walking");
  saved_traj.WriteToFile(FLAGS_data_directory + FLAGS_save_filename);
  std::cout << "Wrote to file: " << FLAGS_data_directory + FLAGS_save_filename
            << std::endl;
  drake::trajectories::PiecewisePolynomial<double> optimal_traj =
      trajopt->ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                         optimal_traj);

  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.5);
    simulator.Initialize();
    simulator.AdvanceTo(optimal_traj.end_time());
  }
}

void setKinematicConstraints(HybridDircon<double>* trajopt,
                             const MultibodyPlant<double>& plant) {
  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  for (const auto& pair : pos_map) {
    std::cout << pair.first << ": " << pair.second << std::endl;
  }

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  // Get the decision variables that will be used
  int N = trajopt->N();
  std::vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points};
  auto x0 = trajopt->initial_state();
  auto ls_mid = trajopt->state(N / 4);
  // Midpoint in the trajectory
  auto x_mid = trajopt->state(N / 2);
  auto rs_mid = trajopt->state(3 * N / 4);
  auto xf = trajopt->final_state();
  auto u = trajopt->input();
  auto u0 = trajopt->input(0);
  auto u_mid = trajopt->input(N / 2);
  auto uf = trajopt->input(N - 1);
  auto x = trajopt->state();

  // Duration Bounds
  double min_duration = (FLAGS_duration > 0.0) ? FLAGS_duration : 2 * 0.2;
  double max_duration = (FLAGS_duration > 0.0) ? FLAGS_duration : 2 * 0.4;

  trajopt->AddDurationBounds(min_duration, max_duration);

  // Standing constraints
  double start_height = FLAGS_start_height;

  // position constraints
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(FLAGS_stride_length, FLAGS_stride_length,
                                    x_mid(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(2 * FLAGS_stride_length,
                                    2 * FLAGS_stride_length,
                                    xf(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_y")));
  trajopt->AddBoundingBoxConstraint(0, 0, x_mid(pos_map.at("base_y")));
  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_y")));

  // initial fb orientation constraint
  VectorXd quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity, x0.head(4));
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity,
                                    x_mid.head(4));
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity, xf.head(4));

  // start height constraints
  trajopt->AddBoundingBoxConstraint(start_height, start_height,
                                    x0(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(start_height, start_height,
                                    x_mid(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(start_height, start_height,
                                    xf(pos_map.at("base_z")));

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
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& asy_joint_name : asy_joint_names) {
      joint_names.push_back(asy_joint_name + l_r_pair.first);
      motor_names.push_back(asy_joint_name + l_r_pair.first + "_motor");
    }
    for (const auto& sym_joint_name : sym_joint_names) {
      joint_names.push_back(sym_joint_name + l_r_pair.first);
      if (sym_joint_name != "ankle_joint") {
        motor_names.push_back(sym_joint_name + l_r_pair.first + "_motor");
      }
    }
  }

  for (const auto& l_r_pair : l_r_pairs) {
    // Symmetry constraints
    for (const auto& sym_joint_name : sym_joint_names) {
      trajopt->AddLinearConstraint(
          x0(pos_map[sym_joint_name + l_r_pair.first]) ==
          xf(pos_map[sym_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_name + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_name + l_r_pair.second + "dot")));
      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddLinearConstraint(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
      }
    }
    // Asymmetry constraints
    for (const auto& asy_joint_name : asy_joint_names) {
      trajopt->AddLinearConstraint(
          x0(pos_map[asy_joint_name + l_r_pair.first]) ==
          -xf(pos_map[asy_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) ==
          -xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")));
      if (asy_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddLinearConstraint(
            u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
            -uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
      }
    }
  }

  // joint limits
  std::cout << "Joint limit constraints: " << std::endl;
  for (const auto& member : joint_names) {
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }

  // actuator limits
  std::cout << "Actuator limit constraints: " << std::endl;
  for (int i = 0; i < trajopt->N(); i++) {
    auto ui = trajopt->input(i);
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  std::cout << "Foot placement constraints: " << std::endl;
  // toe position constraint in y direction (avoid leg crossing)
  // tighter constraint than before
  auto left_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          0.0 * VectorXd::Ones(1), 0.1 * VectorXd::Ones(1));
  auto right_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          -0.1 * VectorXd::Ones(1), 0 * VectorXd::Ones(1));
  for (int mode = 0; mode < 2; ++mode) {
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state((mode_lengths[mode] - 1) * mode + index);
      trajopt->AddConstraint(left_foot_y_constraint, x_i.head(n_q));
      trajopt->AddConstraint(right_foot_y_constraint, x_i.head(n_q));
    }
  }
  // stride length constraint
  auto right_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          (FLAGS_stride_length)*VectorXd::Ones(1),
          (FLAGS_stride_length)*VectorXd::Ones(1));
  auto left_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          (2 * FLAGS_stride_length) * VectorXd::Ones(1),
          (2 * FLAGS_stride_length) * VectorXd::Ones(1));
  trajopt->AddConstraint(right_foot_x_constraint, x_mid.head(n_q));
  trajopt->AddConstraint(left_foot_x_constraint, xf.head(n_q));

//   Foot clearance constraint
  auto left_foot_z_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          (0.1) * VectorXd::Ones(1), (0.1) * VectorXd::Ones(1));
  auto right_foot_z_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          (0.1) * VectorXd::Ones(1), (0.1) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_z_constraint, rs_mid.head(n_q));
  trajopt->AddConstraint(right_foot_z_constraint, ls_mid.head(n_q));

  std::cout << "Adding costs: " << std::endl;
  MatrixXd Q = 0.1 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = 0.005 * MatrixXd::Identity(n_u, n_u);
  Q(0, 0) = 5;
  Q(1, 1) = 5;
  Q(2, 2) = 5;
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  // Add some cost to hip roll and yaw
  double w_q_hip_roll = 0.3;
  double w_q_hip_yaw = 0.3;
  double w_q_hip_pitch = 0.3;
  if (w_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(7, 2);
      trajopt->AddCost(w_q_hip_roll * q.transpose() * q);
    }
  }
  if (w_q_hip_yaw) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(9, 2);
      trajopt->AddCost(w_q_hip_yaw * q.transpose() * q);
    }
  }
  if (w_q_hip_pitch) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(11, 2);
      trajopt->AddCost(w_q_hip_pitch * q.transpose() * q);
    }
  }
}

vector<VectorXd> GetInitGuessForQFlight(int num_knot_points, double apex_height,
                                        const MultibodyPlant<double>& plant) {
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);

  vector<VectorXd> q_init_guess;
  VectorXd q_ik_guess = VectorXd::Zero(n_q);
  Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
  q_ik_guess << quat.normalized(), 0.000889849, 0.000626865, 1.0009, -0.0112109,
      0.00927845, -0.000600725, -0.000895805, 1.15086, 0.610808, -1.38608,
      -1.35926, 0.806192, 1.00716, -M_PI / 2, -M_PI / 2;

  double factor = apex_height / (num_knot_points * num_knot_points / 4.0);
  double rest_height = 1.0;

  for (int i = 0; i < num_knot_points; i++) {
    double eps = 1e-3;
    Vector3d eps_vec = eps * VectorXd::Ones(3);
    double height_offset = apex_height - factor * (i - num_knot_points / 2.0) *
                                             (i - num_knot_points / 2.0);
    Vector3d pelvis_pos(0.0, 0.0, rest_height + height_offset);
    // Do not raise the toes as much as the pelvis, (leg extension)
    Vector3d left_toe_pos(0.0, 0.12, 0.05 + height_offset * 0.5);
    Vector3d right_toe_pos(0.0, -0.12, 0.05 + height_offset * 0.5);

    const auto& world_frame = plant.world_frame();
    const auto& pelvis_frame = plant.GetFrameByName("pelvis");
    const auto& toe_left_frame = plant.GetFrameByName("toe_left");
    const auto& toe_right_frame = plant.GetFrameByName("toe_right");

    drake::multibody::InverseKinematics ik(plant);
    ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0), world_frame,
                             pelvis_pos - eps * VectorXd::Ones(3),
                             pelvis_pos + eps * VectorXd::Ones(3));
    ik.AddOrientationConstraint(pelvis_frame, RotationMatrix<double>(),
                                world_frame, RotationMatrix<double>(), eps);
    ik.AddPositionConstraint(toe_left_frame, Vector3d(0, 0, 0), world_frame,
                             left_toe_pos - eps_vec, left_toe_pos + eps_vec);
    ik.AddPositionConstraint(toe_right_frame, Vector3d(0, 0, 0), world_frame,
                             right_toe_pos - eps_vec, right_toe_pos + eps_vec);
    ik.get_mutable_prog()->AddLinearConstraint(
        (ik.q())(positions_map.at("hip_yaw_left")) == 0);
    ik.get_mutable_prog()->AddLinearConstraint(
        (ik.q())(positions_map.at("hip_yaw_right")) == 0);
    // Four bar linkage constraint (without spring)
    ik.get_mutable_prog()->AddLinearConstraint(
        (ik.q())(positions_map.at("knee_left")) +
            (ik.q())(positions_map.at("ankle_joint_left")) ==
        M_PI * 13 / 180.0);
    ik.get_mutable_prog()->AddLinearConstraint(
        (ik.q())(positions_map.at("knee_right")) +
            (ik.q())(positions_map.at("ankle_joint_right")) ==
        M_PI * 13 / 180.0);

    ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
    const auto result = Solve(ik.prog());
    const auto q_sol = result.GetSolution(ik.q());
    // cout << "  q_sol = " << q_sol.transpose() << endl;
    // cout << "  q_sol.head(4).norm() = " << q_sol.head(4).norm() << endl;
    VectorXd q_sol_normd(n_q);
    q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
    q_ik_guess = q_sol_normd;
    q_init_guess.push_back(q_sol_normd);

    bool visualize_init_traj = false;
    if (visualize_init_traj) {
      // Build temporary diagram for visualization
      drake::systems::DiagramBuilder<double> builder_ik;
      SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<SceneGraph>();
      scene_graph_ik.set_name("scene_graph_ik");
      MultibodyPlant<double> plant_ik(1e-4);
      multibody::addFlatTerrain(&plant_ik, &scene_graph_ik, .8, .8);
      Parser parser(&plant_ik, &scene_graph_ik);
      string full_name =
          FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
      parser.AddModelFromFile(full_name);
      plant_ik.mutable_gravity_field().set_gravity_vector(
          -9.81 * Eigen::Vector3d::UnitZ());
      plant_ik.Finalize();

      // Visualize
      VectorXd x_const = VectorXd::Zero(n_x);
      x_const.head(n_q) = q_sol;
      PiecewisePolynomial<double> pp_xtraj(x_const);

      multibody::connectTrajectoryVisualizer(&plant_ik, &builder_ik,
                                             &scene_graph_ik, pp_xtraj);
      auto diagram = builder_ik.Build();
      drake::systems::Simulator<double> simulator(*diagram);
      simulator.set_target_realtime_rate(.1);
      simulator.Initialize();
      simulator.AdvanceTo(1.0 / num_knot_points);
    }
  }

  return q_init_guess;
}

// Get v by finite differencing q
vector<VectorXd> GetInitGuessForV(const vector<VectorXd>& q_guess, double dt,
                                  const MultibodyPlant<double>& plant) {}

MatrixXd loadSavedDecisionVars(const string& filepath) {
  DirconTrajectory loaded_decision_vars = DirconTrajectory(filepath);
  for (auto& name : loaded_decision_vars.GetTrajectoryNames()) {
    std::cout << name << std::endl;
  }
  return loaded_decision_vars.GetDecisionVariables();
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}