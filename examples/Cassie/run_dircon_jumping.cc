#include <chrono>
#include <memory>
#include <string>

#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/choose_best_solver.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "solvers/nonlinear_cost.h"
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

using dairlib::multibody::CreateWithSpringsToWithoutSpringsMapPos;
using dairlib::multibody::CreateWithSpringsToWithoutSpringsMapVel;
using dairlib::solvers::NonlinearCost;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;
using drake::VectorX;
using drake::math::RotationMatrix;
using drake::multibody::AddMultibodyPlantSceneGraph;
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
DEFINE_double(height, 0.2, "Target height for jumping.");
DEFINE_double(start_height, 1.0, "Starting pelvis height for the trajectory.");
DEFINE_double(distance, 0.0, "Target distance (x) from the initial position.");
DEFINE_double(duration, 0.0, "Duration of the total gait");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");
DEFINE_double(tol, 1e-6, "Tolerance for constraint violation and dual gap");
DEFINE_string(load_filename, "", "File to load decision vars from.");
DEFINE_string(data_directory, "examples/Cassie/saved_trajectories/",
              "Directory path to save decision vars to.");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision "
              "vars to.");
DEFINE_string(traj_name, "", "File to load saved LCM trajs from.");
DEFINE_bool(use_springs, false, "Whether or not to use the spring model");
DEFINE_bool(
    convert_to_springs, false,
    "Whether the loaded trajectory needs to be converted to with springs");
DEFINE_bool(ipopt, false, "Set flag to true to use ipopt instead of snopt");
DEFINE_bool(same_knotpoints, false,
            "Set flag to true if seeding with a trajectory with the same "
            "number of knotpoints");

namespace dairlib {

HybridDircon<double>* createDircon(MultibodyPlant<double>& plant);

void SetKinematicConstraints(HybridDircon<double>* trajopt,
                             const MultibodyPlant<double>& plant);
void AddCosts(HybridDircon<double>* trajopt,
              const MultibodyPlant<double>& plant,
              std::vector<DirconKinematicDataSet<double>*>);
MatrixXd loadSavedDecisionVars(const string& filepath);
void SetInitialGuessFromTrajectory(
    HybridDircon<double>& trajopt, const string& filepath,
    bool same_knot_points = false,
    Eigen::MatrixXd spr_map = Eigen::MatrixXd::Zero(1, 1));


class JointAccelCost : public solvers::NonlinearCost<double> {
 public:
  JointAccelCost(const MatrixXd& W_Q,
                 const drake::multibody::MultibodyPlant<double>& plant,
                 DirconKinematicDataSet<double>* constraints,
                 const std::string& description = "")
    : solvers::NonlinearCost<double>(
    plant.num_positions() + plant.num_velocities() +
      plant.num_actuators() +
      constraints->countConstraintsWithoutSkipping(),
    description),
      plant_(plant),
      context_(plant_.CreateDefaultContext()),
      constraints_(constraints),
      n_v_(plant.num_velocities()),
      n_x_(plant.num_positions() + plant.num_velocities()),
      n_u_(plant.num_actuators()),
      n_lambda_(constraints->countConstraintsWithoutSkipping()),
      W_Q_(W_Q){};

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<double>>& x,
                    drake::VectorX<double>* y) const override {
    DRAKE_ASSERT(x.size() == n_x_ + n_u_ + n_lambda_);

    // Extract our input variables:
    const auto x0 = x.segment(0, n_x_);
    const auto u0 = x.segment(n_x_, n_u_);
    const auto l0 = x.segment(n_x_ + n_u_, n_lambda_);

    multibody::setContext<double>(plant_, x0, u0, context_.get());
    constraints_->updateData(*context_, l0);
    const VectorX<double> xdot0 = constraints_->getXDot();

    (*y) = xdot0.tail(n_v_).transpose() * W_Q_ * xdot0.tail(n_v_);
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
  MultibodyPlant<double> plant(1e-5);

  string file_name =
      "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf";

  if (FLAGS_use_springs) {
    file_name = "examples/Cassie/urdf/cassie_v2.urdf";
  }
  addCassieMultibody(&plant, &scene_graph, true, file_name, FLAGS_use_springs,
                     false);
  plant.Finalize();

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;

  MatrixXd spr_map = MatrixXd::Zero(n_x, n_x);
  if (FLAGS_use_springs && FLAGS_convert_to_springs) {
    MultibodyPlant<double> plant_wo_spr(1e-5);
    Parser parser(&plant_wo_spr);
    parser.AddModelFromFile(
        "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf");
    plant_wo_spr.Finalize();
    spr_map = CreateWithSpringsToWithoutSpringsMapPos(plant, plant_wo_spr);
  }

  std::cout << "nq: " << n_q << " n_v: " << n_v << " n_x: " << n_x << std::endl;
  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  std::cout << "Creating dircon object: " << std::endl;
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
  vector<DirconKinematicData<double>*> double_stance_constraints;
  double_stance_constraints.push_back(&left_toe_front_constraint);
  double_stance_constraints.push_back(&left_toe_rear_constraint);
  double_stance_constraints.push_back(&right_toe_front_constraint);
  double_stance_constraints.push_back(&right_toe_rear_constraint);
  double_stance_constraints.push_back(&distance_constraint_left);
  double_stance_constraints.push_back(&distance_constraint_right);

  vector<DirconKinematicData<double>*> double_stance_land_constraints;
  double_stance_land_constraints.push_back(&left_toe_front_constraint);
  double_stance_land_constraints.push_back(&left_toe_rear_constraint);
  double_stance_land_constraints.push_back(&right_toe_front_constraint);
  double_stance_land_constraints.push_back(&right_toe_rear_constraint);
  double_stance_land_constraints.push_back(&distance_constraint_left);
  double_stance_land_constraints.push_back(&distance_constraint_right);

  auto double_stance_dataset = DirconKinematicDataSet<double>(
      plant, &double_stance_constraints, skip_constraint_inds);
  auto double_stance_land_dataset = DirconKinematicDataSet<double>(
      plant, &double_stance_land_constraints, skip_constraint_inds);
  auto double_stance_options =
      DirconOptions(double_stance_dataset.countConstraints(), plant);
  auto double_stance_land_options =
      DirconOptions(double_stance_land_dataset.countConstraints(), plant);
  /// Be careful setting relative constraint, because we also skip constraints.
  ///                 ||   lf  |   lr  |   rf  |   rr    | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7 8 | 9 10 11 | 12 13
  /// After skipping  || 0 1 2 |   3 4 | 5 6 7 |   8  9  | 10 11
  // Set all the non-z toe constraints to be relative
  double_stance_options.setConstraintRelative(0, true);
  double_stance_options.setConstraintRelative(1, true);
  double_stance_options.setConstraintRelative(3, true);
  double_stance_options.setConstraintRelative(5, true);
  double_stance_options.setConstraintRelative(6, true);
  double_stance_options.setConstraintRelative(8, true);

  double_stance_land_options.setConstraintRelative(0, true);
  double_stance_land_options.setConstraintRelative(1, true);
  double_stance_land_options.setConstraintRelative(2, true);
  double_stance_land_options.setConstraintRelative(3, true);
  double_stance_land_options.setConstraintRelative(4, true);
  double_stance_land_options.setConstraintRelative(5, true);
  double_stance_land_options.setConstraintRelative(6, true);
  double_stance_land_options.setConstraintRelative(7, true);
  double_stance_land_options.setConstraintRelative(8, true);
  double_stance_land_options.setConstraintRelative(9, true);

  //   Flight mode (no contact)
  std::vector<DirconKinematicData<double>*> flight_mode_constraints;
  flight_mode_constraints.push_back(&distance_constraint_left);
  flight_mode_constraints.push_back(&distance_constraint_right);
  auto flight_mode_dataset =
      DirconKinematicDataSet<double>(plant, &flight_mode_constraints);
  auto flight_mode_options =
      DirconOptions(flight_mode_dataset.countConstraints(), plant);

  // timesteps and modes setting
  vector<int> timesteps;  // Number of timesteps per mode
  timesteps.push_back(FLAGS_knot_points);
  timesteps.push_back(FLAGS_knot_points);
  timesteps.push_back(FLAGS_knot_points);
  vector<double> min_dt;  // min/max duration per timestep
  vector<double> max_dt;  // min_duration = knot_points * 0.01 ~= .1s
  min_dt.push_back(.03);
  min_dt.push_back(.03);
  min_dt.push_back(.03);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  // Add contact modes and contact decision
  // variables to single vector for DIRCON
  std::vector<DirconKinematicDataSet<double>*> contact_mode_list;
  std::vector<DirconOptions> options_list;
  contact_mode_list.push_back(&double_stance_dataset);
  contact_mode_list.push_back(&flight_mode_dataset);
  contact_mode_list.push_back(&double_stance_land_dataset);

  options_list.push_back(double_stance_options);
  options_list.push_back(flight_mode_options);
  options_list.push_back(double_stance_land_options);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, timesteps, min_dt, max_dt, contact_mode_list, options_list);

  double tol = FLAGS_tol;
  if (FLAGS_ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    trajopt->SetSolverOption(id, "tol", tol);
    trajopt->SetSolverOption(id, "dual_inf_tol", tol);
    trajopt->SetSolverOption(id, "constr_viol_tol", tol);
    trajopt->SetSolverOption(id, "compl_inf_tol", tol);
    trajopt->SetSolverOption(id, "max_iter", 1e5);
    trajopt->SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt->SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    trajopt->SetSolverOption(id, "print_timing_statistics", "yes");
    trajopt->SetSolverOption(id, "print_level", 5);
    trajopt->SetSolverOption(id, "output_file", "../ipopt.out");

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt->SetSolverOption(id, "acceptable_compl_inf_tol", tol);
    trajopt->SetSolverOption(id, "acceptable_constr_viol_tol", tol);
    trajopt->SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
    trajopt->SetSolverOption(id, "acceptable_tol", 1e2);
    trajopt->SetSolverOption(id, "acceptable_iter", 5);
  } else {
    // Snopt settings
    auto id = drake::solvers::SnoptSolver::id();
    trajopt->SetSolverOption(id, "Print file", "../snopt.out");
    trajopt->SetSolverOption(id, "Major iterations limit", 1e5);
    trajopt->SetSolverOption(id, "Iterations limit", 100000);
    trajopt->SetSolverOption(id, "Verify level", 0);

    // snopt doc said try 2 if seeing snopta exit 40
    trajopt->SetSolverOption(id, "Scale option", 2);
    trajopt->SetSolverOption(id, "Solution", "No");

    // target nonlinear constraint violation
    trajopt->SetSolverOption(id, "Major optimality tolerance", 1e-4);

    // target complementarity gap
    trajopt->SetSolverOption(id, "Major feasibility tolerance", tol);
  }

  std::cout << "Adding kinematic constraints: " << std::endl;
  SetKinematicConstraints(trajopt.get(), plant);
  AddCosts(trajopt.get(), plant, contact_mode_list);
  std::cout << "Setting initial conditions: " << std::endl;
  vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                              FLAGS_knot_points};

  int num_knot_points = trajopt->N();
  std::cout << "nq: " << n_q << endl;
  std::cout << "nv: " << n_v << endl;
  std::cout << "nu: " << plant.num_actuators() << endl;
  cout << "N: " << num_knot_points;
  cout << "Num decision vars: " << trajopt->decision_variables().size() << endl;

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    //    MatrixXd decisionVars =
    //        loadSavedDecisionVars(FLAGS_data_directory + FLAGS_load_filename);
    SetInitialGuessFromTrajectory(*trajopt,
                                  FLAGS_data_directory + FLAGS_load_filename,
                                  FLAGS_same_knotpoints, spr_map);
    //    trajopt->SetInitialGuessForAllVariables(decisionVars);
  }

  // To avoid NaN quaternions
  //  for (int i = 0; i < trajopt->N(); i++) {
  //    auto xi = trajopt->state(i);
  //    if ((trajopt->GetInitialGuess(xi.head(4)).norm() == 0) ||
  //        std::isnan(trajopt->GetInitialGuess(xi.head(4)).norm())) {
  //      trajopt->SetInitialGuess(xi(0), 1);
  //      trajopt->SetInitialGuess(xi(1), 0);
  //      trajopt->SetInitialGuess(xi(2), 0);
  //      trajopt->SetInitialGuess(xi(3), 0);
  //    }
  //  }

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
  DirconTrajectory saved_traj(plant, *trajopt, result, "jumping_trajectory",
                              "Decision variables and state/input trajectories "
                              "for jumping");
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

void SetKinematicConstraints(HybridDircon<double>* trajopt,
                             const MultibodyPlant<double>& plant) {
  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  // Get the decision variables that will be used
  int N = trajopt->N();
  std::vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                                   FLAGS_knot_points};
  auto x0 = trajopt->initial_state();
  auto x_top = trajopt->state(N / 2);
  auto xf = trajopt->final_state();
  auto u = trajopt->input();
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto x = trajopt->state();

  // Duration Bounds
  double min_duration = (FLAGS_duration > 0.0) ? FLAGS_duration : 1.0;
  double max_duration = (FLAGS_duration > 0.0) ? FLAGS_duration : 1.5;

  trajopt->AddDurationBounds(min_duration, max_duration);

  // Standing constraints
  double rest_height = FLAGS_start_height;
  double eps = 1e-6;

  // position constraints
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_y")));
  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_y")));

  // initial fb orientation constraint
  VectorXd quat_identity(4);
  quat_identity << 1, 0, 0, 0;
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity, x0.head(4));
  trajopt->AddBoundingBoxConstraint(quat_identity, quat_identity, xf.head(4));

  // hip yaw and roll constraints
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("hip_yaw_left")));
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("hip_yaw_right")));

  trajopt->AddBoundingBoxConstraint(0.00, 0.1, x0(pos_map.at("hip_roll_left")));
  trajopt->AddBoundingBoxConstraint(-0.1, -0.00,
                                    x0(pos_map.at("hip_roll_right")));

  // hip yaw and roll constraints
  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("hip_yaw_left")));
  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("hip_yaw_right")));

  trajopt->AddBoundingBoxConstraint(-2.0, -1.7, x0(pos_map.at("toe_left")));
  trajopt->AddBoundingBoxConstraint(-2.0, -1.7, x0(pos_map.at("toe_right")));
  trajopt->AddBoundingBoxConstraint(-2.0, -1.7, xf(pos_map.at("toe_left")));
  trajopt->AddBoundingBoxConstraint(-2.0, -1.7, xf(pos_map.at("toe_right")));

  // Jumping height constraints
  trajopt->AddBoundingBoxConstraint(rest_height - eps, rest_height + eps,
                                    x0(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(FLAGS_height + rest_height - eps,
                                    FLAGS_height + rest_height + eps,
                                    x_top(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(0.8 * FLAGS_height + rest_height - eps,
                                    0.8 * FLAGS_height + rest_height + eps,
                                    xf(pos_map.at("base_z")));

  // Zero starting and final velocities
  trajopt->AddLinearConstraint(VectorXd::Zero(n_v) == x0.tail(n_v));
  trajopt->AddLinearConstraint(VectorXd::Zero(n_v) == xf.tail(n_v));

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
  l_r_pairs.pop_back();

  // Symmetry constraints
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      trajopt->AddConstraintToAllKnotPoints(
          x0(pos_map[sym_joint_name + l_r_pair.first]) ==
          x0(pos_map[sym_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          xf(pos_map[sym_joint_name + l_r_pair.first]) ==
          xf(pos_map[sym_joint_name + l_r_pair.second]));
      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddConstraintToAllKnotPoints(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            u0(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
        trajopt->AddConstraintToAllKnotPoints(
            uf(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
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
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -175),
                                      VectorXd::Constant(n_u, +175), ui);
  }

  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);

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
  //  for (int mode = 2; mode < 3; ++mode) {
  for (int mode : {0, 1, 2}) {
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state((mode_lengths[mode] - 1) * mode + index);
      trajopt->AddConstraint(left_foot_y_constraint, x_i.head(n_q));
      trajopt->AddConstraint(right_foot_y_constraint, x_i.head(n_q));
    }
  }

  // Jumping distance constraint for platform clearance
  auto left_foot_x_start_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          (0 - eps) * VectorXd::Ones(1), (0 + eps) * VectorXd::Ones(1));
  auto right_foot_x_start_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          (0 - eps) * VectorXd::Ones(1), (0 + eps) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_x_start_constraint, x0.head(n_q));
  trajopt->AddConstraint(right_foot_x_start_constraint, x0.head(n_q));

  // Jumping distance constraint for platform clearance
  auto left_foot_x_platform_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          0.33 * (FLAGS_distance - eps) * VectorXd::Ones(1),
          0.33 * (FLAGS_distance + eps) * VectorXd::Ones(1));
  auto right_foot_x_platform_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(1, 0, 0),
          0.33 * (FLAGS_distance - eps) * VectorXd::Ones(1),
          0.33 * (FLAGS_distance + eps) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_x_platform_constraint, x_top.head(n_q));
  trajopt->AddConstraint(right_foot_x_platform_constraint, x_top.head(n_q));

  // Jumping distance constraint
  auto left_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          (FLAGS_distance - eps) * VectorXd::Ones(1),
          (FLAGS_distance + eps) * VectorXd::Ones(1));
  auto right_foot_x_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(1, 0, 0),
          (FLAGS_distance - eps) * VectorXd::Ones(1),
          (FLAGS_distance + eps) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_x_constraint, xf.head(n_q));
  trajopt->AddConstraint(right_foot_x_constraint, xf.head(n_q));

  // Foot clearance constraint
  auto left_foot_z_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          (1.25 * FLAGS_height - eps) * VectorXd::Ones(1),
          (1.25 * FLAGS_height + eps) * VectorXd::Ones(1));
  auto right_foot_z_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 0, 1),
          (1.25 * FLAGS_height - eps) * VectorXd::Ones(1),
          (1.25 * FLAGS_height + eps) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_z_constraint, x_top.head(n_q));
  trajopt->AddConstraint(right_foot_z_constraint, x_top.head(n_q));

  auto left_foot_rear_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_rear_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  auto right_foot_rear_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_rear_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_rear_z_final_constraint, xf.head(n_q));
  trajopt->AddConstraint(right_foot_rear_z_final_constraint, xf.head(n_q));

  auto left_foot_front_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_left", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  auto right_foot_front_z_final_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", pt_front_contact, Eigen::RowVector3d(0, 0, 1),
          (FLAGS_height - eps) * VectorXd::Ones(1),
          (FLAGS_height + eps) * VectorXd::Ones(1));
  trajopt->AddConstraint(left_foot_front_z_final_constraint, xf.head(n_q));
  trajopt->AddConstraint(right_foot_front_z_final_constraint, xf.head(n_q));

  // Only add constraints of lambdas for stance modes
  vector<int> stance_modes{0, 2};
  // ALL BUT THE LAST SEGMENT (to ensure the feet can leave the ground
  for (int index = 0; index < (mode_lengths[0] - 1); index++) {
    auto lambda = trajopt->force(0, index);
    trajopt->AddLinearConstraint(lambda(2) >= 10);
    trajopt->AddLinearConstraint(lambda(5) >= 10);
    trajopt->AddLinearConstraint(lambda(8) >= 10);
    trajopt->AddLinearConstraint(lambda(11) >= 10);
  }
  // Limit the ground reaction forces in the landing phase
  for (int index = 0; index < mode_lengths[2]; index++) {
    auto lambda = trajopt->force(2, index);
    trajopt->AddLinearConstraint(lambda(2) <= 300);
    trajopt->AddLinearConstraint(lambda(5) <= 300);
    trajopt->AddLinearConstraint(lambda(8) <= 300);
    trajopt->AddLinearConstraint(lambda(11) <= 300);
  }
}

/******
COSTS
******/
void AddCosts(HybridDircon<double>* trajopt,
              const MultibodyPlant<double>& plant,
              std::vector<DirconKinematicDataSet<double>*> constraints) {
  auto x = trajopt->state();
  auto u = trajopt->input();
  int N = trajopt->N();

  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

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
  l_r_pairs.pop_back();

  double w_symmetry_pos = 10.0;
  double w_symmetry_vel = 500.0;
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      auto pos_diff = x(pos_map[sym_joint_name + l_r_pair.first]) -
                      x(pos_map[sym_joint_name + l_r_pair.second]);
      auto vel_diff = x(pos_map[sym_joint_name + l_r_pair.first]) -
                      x(pos_map[sym_joint_name + l_r_pair.second]);
      trajopt->AddRunningCost(w_symmetry_pos * pos_diff * pos_diff);
      trajopt->AddRunningCost(w_symmetry_vel * vel_diff * vel_diff);
    }
  }
  MatrixXd Q = 0.01 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = 0.001 * MatrixXd::Identity(n_u, n_u);
  Q(0, 0) = 1;
  Q(1, 1) = 5.0;
  Q(2, 2) = 1;
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  // Add some cost to hip roll and yaw
  double w_q_hip_roll = 0.0;
  double w_q_hip_yaw = 500.0;
  double w_q_hip_pitch = 0.0;
  double w_q_quat_xyz = 0.0;
  if (w_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(7, 2);
      trajopt->AddCost(w_q_hip_roll * q.transpose() * q);
    }
  }
//  if (w_q_hip_yaw) {
  for (int i = 0; i < N; i++) {
//      auto q = trajopt->state(i).segment(9, 2);
    auto q = trajopt->state(i).segment(27, 2);
    trajopt->AddCost(w_q_hip_yaw * q.transpose() * q);
  }
//  }
  if (w_q_hip_pitch) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(11, 2);
      trajopt->AddCost(w_q_hip_pitch * q.transpose() * q);
    }
  }
  if (w_q_quat_xyz) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(1, 3);
      trajopt->AddCost(w_q_quat_xyz * q.transpose() * q);
    }
  }

  vector<int> mode_lengths = {FLAGS_knot_points, FLAGS_knot_points,
                              FLAGS_knot_points};
  Q = 1e-4 * MatrixXd::Identity(n_v, n_v);
  Q(21, 21) = 1e-6;
  Q(19, 19) = 1e-6;
  vector<std::shared_ptr<JointAccelCost>> joint_accel_costs;
//  trajopt->AddCost(joint_accel_cost, );
//  int n_mode = joint_accel_cost_in_second_mode ? num_time_samples.size() : 1;
//  int idx_start = 0;
//  for (unsigned int mode = 0; mode < n_mode; mode++) {
//    for (int index = 0; index < num_time_samples[mode]; index++) {
//      auto xi = trajopt->state_vars_by_mode(mode, index);
//      auto ui = trajopt->input(mode, index);
//      auto li = trajopt->force(mode, index);
//      trajopt->AddCost(joint_accel_cost, {xi, ui, li}));
//    }
//    idx_start += num_time_samples[mode] - 1;
//  }
  for (int mode : {0, 1, 2}) {
    joint_accel_costs.push_back(std::make_shared<JointAccelCost>(
      Q, plant, constraints[mode]));
    for (int index = 0; index < mode_lengths[mode]; index++) {
      // Assumes mode_lengths are the same across modes
      auto x_i = trajopt->state_vars_by_mode(mode, index);
      auto u_i = trajopt->input((mode_lengths[mode] - 1) * mode + index);
      auto l_i = trajopt->force(mode, index);
      trajopt->AddCost(joint_accel_costs[mode], {x_i, u_i, l_i});
    }
  }
}

void SetInitialGuessFromTrajectory(HybridDircon<double>& trajopt,
                                   const string& filepath,
                                   bool same_knot_points,
                                   Eigen::MatrixXd spr_map) {
  DirconTrajectory previous_traj = DirconTrajectory(filepath);
  if (same_knot_points) {
    trajopt.SetInitialGuessForAllVariables(
        previous_traj.GetDecisionVariables());
    return;
  }
  PiecewisePolynomial<double> state_traj;
  if (FLAGS_use_springs && FLAGS_convert_to_springs) {
    std::cout << "Using spring conversion" << std::endl;
    state_traj = previous_traj.ReconstructStateTrajectoryWithSprings(spr_map);
  } else {
    state_traj = previous_traj.ReconstructStateTrajectory();
  }
  auto input_traj = previous_traj.ReconstructInputTrajectory();
  auto lambda_traj = previous_traj.ReconstructLambdaTrajectory();
  auto lambda_c_traj = previous_traj.ReconstructLambdaCTrajectory();
  auto gamma_traj = previous_traj.ReconstructGammaCTrajectory();

  trajopt.SetInitialTrajectory(input_traj, state_traj);
  for (int mode = 0; mode < trajopt.num_modes(); ++mode) {
    trajopt.SetInitialForceTrajectory(mode, lambda_traj[mode],
                                      lambda_c_traj[mode], gamma_traj[mode]);
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