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
DEFINE_int32(scale_option, 2,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");
DEFINE_bool(scale_variables, true, "Set to false if using default scaling");
DEFINE_bool(scale_constraints, true, "Set to false if using default scaling");
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

void setKinematicConstraints(HybridDircon<double>* trajopt,
                             const MultibodyPlant<double>& plant,
                             std::vector<DirconKinematicDataSet<double>*>);
vector<VectorXd> GetInitGuessForQStance(int num_knot_points,
                                        const MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForQFlight(int num_knot_points, double apex_height,
                                        const MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForV(const vector<VectorXd>& q_guess, double dt,
                                  const MultibodyPlant<double>& plant);
MatrixXd loadSavedDecisionVars(const string& filepath);
void SetInitialGuessFromTrajectory(
    const shared_ptr<HybridDircon<double>>& trajopt, const string& filepath);

vector<string> createStateNameVectorFromMap(const map<string, int>& pos_map,
                                            const map<string, int>& vel_map,
                                            const map<string, int>& act_map);
class JointAccelConstraint : public solvers::NonlinearConstraint<double> {
 public:
  JointAccelConstraint(const VectorXd& lb, const VectorXd& ub,
                       const drake::multibody::MultibodyPlant<double>& plant,
                       DirconKinematicDataSet<double>* constraints,
                       const std::string& description = "")
      : solvers::NonlinearConstraint<double>(
            plant.num_velocities(),
            plant.num_positions() + plant.num_velocities() +
                plant.num_actuators() +
                constraints->countConstraintsWithoutSkipping(),
            lb, ub, description),
        plant_(plant),
        context_(plant_.CreateDefaultContext()),
        constraints_(constraints),
        n_v_(plant.num_velocities()),
        n_x_(plant.num_positions() + plant.num_velocities()),
        n_u_(plant.num_actuators()),
        n_lambda_(constraints->countConstraintsWithoutSkipping()){};

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    DRAKE_ASSERT(x.size() == n_x_ + n_u_ + n_lambda_);

    // Extract our input variables:
    const auto x0 = x.segment(0, n_x_);
    const auto u0 = x.segment(n_x_, n_u_);
    const auto l0 = x.segment(n_x_ + n_u_, n_lambda_);

    multibody::setContext<double>(plant_, x0, u0, context_.get());
    constraints_->updateData(*context_, l0);

    (*y) = constraints_->getXDot().tail(n_v_);
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  DirconKinematicDataSet<double>* constraints_;

  int n_v_;
  int n_x_;
  int n_u_;
  int n_lambda_;

  MatrixXd W_Q_;
};

void DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(0.0);
  //  Parser parser(&plant, &scene_graph);

  string file_name = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  addCassieMultibody(&plant, &scene_graph, true, file_name, false, false);
  plant.Finalize();

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
  timesteps.push_back(1);
  vector<double> min_dt;  // min/max duration per timestep
  vector<double> max_dt;  // min_duration = knot_points * 0.01 ~= .1s
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  // Add contact modes and contact decision
  // variables to single vector for DIRCON
  std::vector<DirconKinematicDataSet<double>*> contact_mode_list;
  std::vector<DirconOptions> options_list;
  contact_mode_list.push_back(&left_stance_dataset);
  contact_mode_list.push_back(&right_stance_dataset);
  contact_mode_list.push_back(&left_stance_dataset);

  options_list.push_back(left_stance_options);
  options_list.push_back(right_stance_options);
  options_list.push_back(left_stance_options);

  if (FLAGS_scale_constraints) {
    for (int i = 0; i < options_list.size(); i++) {
      double s = 1;  // scale everything together
      // Dynamic constraints
      options_list[i].setDynConstraintScaling({0, 1, 2, 3}, s * 1.0 / 30.0);
      options_list[i].setDynConstraintScaling(
          {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}, s * 1.0 / 60.0);
      options_list[i].setDynConstraintScaling({17, 18}, s * 1.0 / 300.0);
      options_list[i].setDynConstraintScaling(
          {19, 20, 21, 22, 23, 24, 25, 26, 27, 28}, s * 1.0 / 600.0);
      options_list[i].setDynConstraintScaling({29, 30, 31, 32, 33, 34},
                                              s * 1.0 / 3000.0);
      options_list[i].setDynConstraintScaling({35, 36}, s * 1.0 / 60000.0);
      // Kinematic constraints
      int n_l = options_list[i].getNumConstraints();
      options_list[i].setKinConstraintScaling({0, 1, 2, 3, 4}, s / 6000.0);
      options_list[i].setKinConstraintScaling(
          {n_l + 0, n_l + 1, n_l + 2, n_l + 3, n_l + 4}, s / 10.0);
      options_list[i].setKinConstraintScaling(
          {2 * n_l + 0, 2 * n_l + 1, 2 * n_l + 2, 2 * n_l + 3, 2 * n_l + 4}, s);
      options_list[i].setKinConstraintScaling({5, 6}, s / 300.0);
      options_list[i].setKinConstraintScaling({n_l + 5, n_l + 6}, s);
      options_list[i].setKinConstraintScaling({2 * n_l + 5, 2 * n_l + 6},
                                              s * 20);
      // Impact constraints
      options_list[i].setImpConstraintScaling({0, 1, 2}, s / 50.0);
      options_list[i].setImpConstraintScaling({3, 4, 5}, s / 300.0);
      options_list[i].setImpConstraintScaling({6, 7}, s / 24.0);
      options_list[i].setImpConstraintScaling({8, 9}, s / 6.0);
      options_list[i].setImpConstraintScaling({10, 11, 12, 13}, s / 12.0);
      options_list[i].setImpConstraintScaling({14, 15}, s / 2.0);
      options_list[i].setImpConstraintScaling({16, 17}, s);
    }
  }

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, timesteps, min_dt, max_dt, contact_mode_list, options_list);


  if (FLAGS_scale_variables) {
    std::vector<int> idx_list;
    // time
    trajopt->ScaleTimeVariables(0.008);
    // state
    trajopt->ScaleStateVariables({0, 1, 2, 3}, 0.5);
    idx_list.clear();
    for (int i = n_q; i < n_q + n_v - 2; i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleStateVariables(idx_list, 10);
    trajopt->ScaleStateVariable(n_q + n_v - 2, 10);
    trajopt->ScaleStateVariable(n_q + n_v - 1, 10);
    // input
    trajopt->ScaleInputVariables({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, 100);
    // force
    idx_list.clear();
    for (int i = 0; i < left_stance_dataset.countConstraintsWithoutSkipping();
         i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleForceVariables(0, idx_list, 1000);
    idx_list.clear();
    for (int i = 0; i < right_stance_dataset.countConstraintsWithoutSkipping();
         i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleForceVariables(1, idx_list, 1000);
    // impulse
    trajopt->ScaleImpulseVariables(0, idx_list, 10);
    // quaternion slack
    trajopt->ScaleQuaternionSlackVariables(30);
    // Constraint slack
    trajopt->ScaleKinConstraintSlackVariables(0, {0, 1, 2, 3, 4, 5}, 50);
    trajopt->ScaleKinConstraintSlackVariables(0, {6, 7}, 500);
  }

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../walking_snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 100000);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Iterations limit", 100000);  // QP subproblems
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
  setKinematicConstraints(trajopt.get(), plant, contact_mode_list);
  std::cout << "Setting initial conditions: " << std::endl;

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    SetInitialGuessFromTrajectory(trajopt,
                                  FLAGS_data_directory + FLAGS_load_filename);
    //    MatrixXd decisionVars =
    //        loadSavedDecisionVars(FLAGS_data_directory + FLAGS_load_filename);
    //    trajopt->SetInitialGuessForAllVariables(decisionVars);
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
  int num_poses = std::min(FLAGS_knot_points, 10);
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
                             const MultibodyPlant<double>& plant,
                             std::vector<DirconKinematicDataSet<double>*> constraints) {
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
  std::vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points, 1};
  auto x0 = trajopt->initial_state();
  auto ls_mid = trajopt->state(FLAGS_knot_points / 2);
  // Midpoint in the trajectory
  auto x_mid = trajopt->state(N / 2);
  auto rs_mid = trajopt->state(FLAGS_knot_points + FLAGS_knot_points / 2);
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

  //acceleration_constraints
//  int n = 0;
//  for (int mode = 0; mode < mode_lengths.size(); ++mode){
//    for (int i = 0; i < mode_lengths[mode]; ++i){
//      auto x_i = trajopt->state_vars_by_mode(mode, i);
//      auto u_i = trajopt->input(n + i);
//      auto lambda_i = trajopt->force(mode, i);
//      VectorXd lb = -2 * VectorXd::Ones(n_v);
//      VectorXd ub = 2 * VectorXd::Ones(n_v);
//      auto acceleration_constraint = std::make_shared<JointAccelConstraint>(
//          lb, ub, plant, constraints[mode], "left_stance_accel_constraints");
//      trajopt->AddConstraint(acceleration_constraint, {x_i, u_i, lambda_i});
//    }
//    n += mode_lengths[mode] - 1;
//  }

  // position constraints
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(FLAGS_stride_length, FLAGS_stride_length,
                                    x_mid(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(2 * FLAGS_stride_length,
                                    2 * FLAGS_stride_length,
                                    xf(pos_map.at("base_x")));

  //  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_y")));
  //  trajopt->AddBoundingBoxConstraint(0, 0, x_mid(pos_map.at("base_y")));
  //  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_y")));

  // initial fb orientation constraint
  VectorXd quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity, x0.head(4));
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity, xf.head(4));

  trajopt->AddLinearConstraint(x0(pos_map.at("base_qw")) ==
                               x_mid(pos_map.at("base_qw")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qx")) ==
                               -x_mid(pos_map.at("base_qx")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qy")) ==
                               x_mid(pos_map.at("base_qy")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qz")) ==
                               -x_mid(pos_map.at("base_qz")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_y")) ==
                               -x_mid(pos_map.at("base_y")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
                               x_mid(n_q + vel_map.at("base_wx")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
                               -x_mid(n_q + vel_map.at("base_wy")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
                               x_mid(n_q + vel_map.at("base_wz")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
                               x_mid(n_q + vel_map.at("base_vx")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
                               -x_mid(n_q + vel_map.at("base_vy")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
                               x_mid(n_q + vel_map.at("base_vz")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_y")) ==
                               xf(pos_map.at("base_y")));
  //  trajopt->AddLinearConstraint(x0.tail(n_v) == xf.tail(n_v));

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
  vector<string> asy_joint_names{"hip_roll", "hip_yaw"};
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
          x_mid(pos_map[sym_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_name + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_name + l_r_pair.second + "dot")));
      trajopt->AddLinearConstraint(
          x0(pos_map[sym_joint_name + l_r_pair.first]) ==
          x_mid(pos_map[sym_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_name + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_name + l_r_pair.second + "dot")));
      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddLinearConstraint(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            u_mid(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
        trajopt->AddLinearConstraint(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
      }
    }
    // Asymmetry constraints
    for (const auto& asy_joint_name : asy_joint_names) {
      trajopt->AddLinearConstraint(
          x0(pos_map[asy_joint_name + l_r_pair.first]) ==
          -x_mid(pos_map[asy_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          x0(pos_map[asy_joint_name + l_r_pair.first]) ==
          xf(pos_map[asy_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) ==
          -x_mid(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")));
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")));
      if (asy_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddLinearConstraint(
            u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
            -u_mid(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
        trajopt->AddLinearConstraint(
            u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
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
          0.1 * VectorXd::Ones(1), 0.15 * VectorXd::Ones(1));
  auto right_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          -0.15 * VectorXd::Ones(1), -0.1 * VectorXd::Ones(1));
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
          (FLAGS_stride_length - 0.02) * VectorXd::Ones(1),
          (FLAGS_stride_length + 0.02) * VectorXd::Ones(1));
  auto left_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          (2 * FLAGS_stride_length - 0.02) * VectorXd::Ones(1),
          (2 * FLAGS_stride_length + 0.02) * VectorXd::Ones(1));
  trajopt->AddConstraint(right_foot_x_constraint, x_mid.head(n_q));
  trajopt->AddConstraint(left_foot_x_constraint, xf.head(n_q));

  //   Foot clearance constraint
  auto left_foot_z_constraint_clearance =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          0 * VectorXd::Ones(1), (0.15) * VectorXd::Ones(1));
  auto right_foot_z_constraint_clearance =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          0 * VectorXd::Ones(1), (0.15) * VectorXd::Ones(1));
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
  for (int i = 0; i < N; i++) {
    auto x_i = trajopt->state(i);
    trajopt->AddConstraint(left_foot_z_constraint_clearance, x_i.head(n_q));
    trajopt->AddConstraint(right_foot_z_constraint_clearance, x_i.head(n_q));
    trajopt->AddBoundingBoxConstraint(start_height - 0.10, start_height + 0.10, x_i[pos_map["base_z"]]);
    trajopt->AddBoundingBoxConstraint(-2.0, -1.5, x_i[pos_map["toe_left"]]);
    trajopt->AddBoundingBoxConstraint(-2.0, -1.5, x_i[pos_map["toe_right"]]);

  }
  for (unsigned int mode = 0; mode < mode_lengths.size(); mode++) {
    for (int index = 0; index < mode_lengths[mode]; index++) {
      auto lambda = trajopt->force(mode, index);
      trajopt->AddLinearConstraint(lambda(2) >= 10);
      trajopt->AddLinearConstraint(lambda(5) >= 10);
    }
  }

  std::cout << "Adding costs: " << std::endl;
  MatrixXd Q = 1e-8 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = 0.001 * MatrixXd::Identity(n_u, n_u);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);
  MatrixXd Q0 = MatrixXd::Identity(n_v, n_v);
  trajopt->AddQuadraticCost(x0.tail(n_v).transpose() * Q * x0.tail(n_v));
  // Add some cost to hip roll and yaw
  double w_q_hip_roll = 0.3;
  double w_q_hip_yaw = 0.3;
  double w_q_hip_pitch = 2.0;
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

void SetInitialGuessFromTrajectory(
    const shared_ptr<HybridDircon<double>>& trajopt, const string& filepath) {
  DirconTrajectory previous_traj = DirconTrajectory(filepath);
  auto state_traj = previous_traj.ReconstructStateTrajectory();
  auto input_traj = previous_traj.ReconstructInputTrajectory();
  auto lambda_traj = previous_traj.ReconstructLambdaTrajectory();
  auto lambda_c_traj = previous_traj.ReconstructLambdaCTrajectory();
  auto gamma_traj = previous_traj.ReconstructGammaCTrajectory();

  trajopt->SetInitialTrajectory(input_traj, state_traj);
  for (int mode = 0; mode < trajopt->num_modes() - 1; ++mode) {
    if (trajopt->mode_lengths()[mode] > 1) {
      std::cout << "mode: " << mode << std::endl;
      trajopt->SetInitialForceTrajectory(mode, lambda_traj[mode],
                                         lambda_c_traj[mode], gamma_traj[mode]);
    }
  }
}

MatrixXd loadSavedDecisionVars(const string& filepath) {
  DirconTrajectory previous_traj = DirconTrajectory(filepath);
  for (auto& name : previous_traj.GetTrajectoryNames()) {
    std::cout << name << std::endl;
  }
  return previous_traj.GetDecisionVariables();
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}