#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using dairlib::systems::trajectory_optimization::PointPositionConstraint;
using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::solvers::Constraint;
using drake::solvers::SolutionResult;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::trajectories::PiecewisePolynomial;

DEFINE_string(data_directory, "default_filepath",
              "directory to save/read data");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision "
              "vars and trajectories to.");
DEFINE_string(load_filename, "default_filename",
              "Filename to load decision "
              "vars from.");
// SNOPT parameters
DEFINE_int32(max_iter, 100000, "Iteration limit");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");

// Gait and traj opt parameters
DEFINE_int32(knot_points, 12, "Number of nodes");
DEFINE_bool(fix_time, true, "Whether to fix the duration of gait or not");
DEFINE_double(duration, 0.4,
              "Duration of the single support phase (s)."
              "If is_fix_time = false, then duration is only used in initial "
              "guess calculation");
DEFINE_double(stride_length, 0.2, "stride length of the walking");
DEFINE_double(ground_incline, 0.0,
              "incline level of the ground"
              "(not implemented yet)");

// Parameters which enable scaling to improve solving speed
DEFINE_bool(is_scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(is_scale_variable, true, "Scale the decision variable");

// Others
DEFINE_bool(visualize_init_guess, false,
            "to visualize the poses of the initial guess");

namespace dairlib {

/// Trajectory optimization of fixed-spring cassie walking

vector<VectorXd> GetInitGuessForQ(int num_knot_points, double stride_length,
                                  double ground_incline,
                                  const MultibodyPlant<double>& plant,
                                  bool visualize_init_guess = false);
vector<VectorXd> GetInitGuessForV(const vector<VectorXd>& q_guess, double dt,
                                  const MultibodyPlant<double>& plant);
MatrixXd loadSavedDecisionVars(const string& filepath);
MatrixXd generateStateAndInputMatrix(const PiecewisePolynomial<double>& states,
                                     const PiecewisePolynomial<double>& inputs,
                                     VectorXd times);

void DoMain() {
  // Dircon parameter
  double minimum_timestep = 0.01;
  DRAKE_DEMAND(FLAGS_duration / (FLAGS_knot_points - 1) >= minimum_timestep);
  // If the node density is too low, it's harder for SNOPT to converge well.
  double max_distance_per_node = 0.2 / 16;
  DRAKE_DEMAND((FLAGS_stride_length / FLAGS_knot_points) <=
               max_distance_per_node);

  // Cost on velocity and input
  double w_Q = 0.05;
  double w_R = 0.0001;
  // Cost on force
  double w_lambda = sqrt(0.1) * 1.0e-4;
  // Cost on difference over time
  double w_lambda_diff = 0.00000001;
  double w_v_diff = 0.0005;
  double w_u_diff = 0.0000001;
  // Cost on position
  double w_q_hip_roll = 5;
  double w_q_hip_yaw = 5;
  double w_q_quat_xyz = 5;

  // Optional constraints
  // This seems to be important at higher walking speeds
  bool constrain_stance_leg_fourbar_force = false;

  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);

  Vector3d ground_normal(sin(FLAGS_ground_incline), 0,
                         cos(FLAGS_ground_incline));
  multibody::addFlatTerrain(&plant, &scene_graph, 1, 1, ground_normal);

  Parser parser(&plant, &scene_graph);

  string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  plant.Finalize();

  // Create maps for joints
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

  // Set up contact/distance constraints
  const Body<double>& toe_left = plant.GetBodyByName("toe_left");
  const Body<double>& toe_right = plant.GetBodyByName("toe_right");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  bool isXZ = false;
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
  vector<int> ds_skip_constraint_inds;
  ds_skip_constraint_inds.push_back(3);
  ds_skip_constraint_inds.push_back(9);

  // Left support
  vector<DirconKinematicData<double>*> ls_constraint;
  ls_constraint.push_back(&left_toe_front_constraint);
  ls_constraint.push_back(&left_toe_rear_constraint);
  ls_constraint.push_back(&distance_constraint_left);
  ls_constraint.push_back(&distance_constraint_right);
  auto ls_dataset = DirconKinematicDataSet<double>(plant, &ls_constraint,
                                                   skip_constraint_inds);
  auto ls_options = DirconOptions(ls_dataset.countConstraints(), plant);

  ls_options.setConstraintRelative(0, true);
  ls_options.setConstraintRelative(1, true);
  ls_options.setConstraintRelative(3, true);

  // Right support
  vector<DirconKinematicData<double>*> rs_constraint;
  rs_constraint.push_back(&right_toe_front_constraint);
  rs_constraint.push_back(&right_toe_rear_constraint);
  rs_constraint.push_back(&distance_constraint_left);
  rs_constraint.push_back(&distance_constraint_right);
  auto rs_dataset = DirconKinematicDataSet<double>(plant, &rs_constraint,
                                                   skip_constraint_inds);

  // Double support
  vector<DirconKinematicData<double>*> ds_constraint;
  ds_constraint.push_back(&left_toe_front_constraint);
  ds_constraint.push_back(&left_toe_rear_constraint);
  ds_constraint.push_back(&right_toe_front_constraint);
  ds_constraint.push_back(&right_toe_rear_constraint);
  ds_constraint.push_back(&distance_constraint_left);
  ds_constraint.push_back(&distance_constraint_right);
  auto ds_dataset = DirconKinematicDataSet<double>(plant, &ds_constraint,
                                                   ds_skip_constraint_inds);
  auto ds_options = DirconOptions(ds_dataset.countConstraints(), plant);

  ds_options.setConstraintRelative(0, true);
  ds_options.setConstraintRelative(1, true);
  ds_options.setConstraintRelative(3, true);
  ds_options.setConstraintRelative(5, true);
  ds_options.setConstraintRelative(6, true);
  ds_options.setConstraintRelative(8, true);

  // Set up options
  vector<DirconKinematicDataSet<double>*> dataset_list;
  std::vector<DirconOptions> options_list;

  dataset_list.push_back(&ds_dataset);
  dataset_list.push_back(&ls_dataset);
  dataset_list.push_back(&ds_dataset);

  options_list.push_back(ds_options);
  options_list.push_back(ls_options);
  options_list.push_back(ds_options);

  // set force cost weight
  for (int i = 0; i < 3; i++) {
    options_list[i].setForceCost(w_lambda);
  }

  // Be careful in setting relative constraint, because we skip constraints
  ///                 || lf/rf | lr/rr | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7
  /// After skipping  || 0 1 2 |   3 4 | 5 6

  // Constraint scaling
  if (FLAGS_is_scale_constraint) {
    for (int i = 0; i < 3; i++) {
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

  // timesteps and modes setting
  vector<double> min_dt;
  vector<double> max_dt;
  min_dt.push_back(minimum_timestep);
  min_dt.push_back(minimum_timestep);
  min_dt.push_back(minimum_timestep);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  vector<int> timesteps;
  timesteps.push_back(FLAGS_knot_points / 4);
  timesteps.push_back(FLAGS_knot_points);
  timesteps.push_back(FLAGS_knot_points / 4);
  //  timesteps.push_back(1);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, timesteps, min_dt, max_dt, dataset_list, options_list);

  // Snopt settings
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", FLAGS_max_iter);
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

  int N = trajopt->N();

  std::cout << "N: " << N << std::endl;

  // Get the decision variables that will be used
  auto u = trajopt->input();
  auto x = trajopt->state();
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->final_state();
  vector<int> mode_lengths(timesteps);

  // Fix time FLAGS_duration
  if (FLAGS_fix_time) {
    trajopt->AddDurationBounds(FLAGS_duration, FLAGS_duration);
  }

  // x position constraint
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(FLAGS_stride_length, FLAGS_stride_length,
                                    xf(pos_map.at("base_x")));

  // Floating base periodicity
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qw")) ==
                               xf(pos_map.at("base_qw")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qx")) ==
                               -xf(pos_map.at("base_qx")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qy")) ==
                               xf(pos_map.at("base_qy")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_qz")) ==
                               -xf(pos_map.at("base_qz")));
  trajopt->AddLinearConstraint(x0(pos_map.at("base_y")) ==
                               -xf(pos_map.at("base_y")));
  if (FLAGS_ground_incline == 0) {
    trajopt->AddLinearConstraint(x0(pos_map.at("base_z")) ==
                                 xf(pos_map.at("base_z")));
  }
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
                               xf(n_q + vel_map.at("base_wx")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
                               -xf(n_q + vel_map.at("base_wy")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
                               xf(n_q + vel_map.at("base_wz")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
                               xf(n_q + vel_map.at("base_vx")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
                               -xf(n_q + vel_map.at("base_vy")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
                               xf(n_q + vel_map.at("base_vz")));

  // The legs joint positions/velocities/torque should be mirrored between legs
  // (notice that hip yaw and roll should be asymmetric instead of symmetric.)
  for (const auto& l_r_pair : l_r_pairs) {
    for (auto asy_joint_name : asy_joint_names) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(asy_joint_name + l_r_pair.first)) ==
          -xf(pos_map.at(asy_joint_name + l_r_pair.second)));
      // velocities
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_name + l_r_pair.first + "dot")) ==
          -xf(n_q + vel_map.at(asy_joint_name + l_r_pair.second + "dot")));
      // inputs
      trajopt->AddLinearConstraint(
          u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
          -uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
    }
    //    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
    for (auto sym_joint_name : sym_joint_names) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(sym_joint_name + l_r_pair.first)) ==
          xf(pos_map.at(sym_joint_name + l_r_pair.second)));
      // velocities
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_name + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_name + l_r_pair.second + "dot")));
      // inputs (ankle joint is not actuated)
      if (sym_joint_name != "ankle_joint") {
        trajopt->AddLinearConstraint(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
      }
    }
  }  // end for (l_r_pairs)

  // joint limits
  for (const auto& member : joint_names) {
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) <=
        plant.GetJointByName(member).position_upper_limits()(0));
    trajopt->AddConstraintToAllKnotPoints(
        x(pos_map.at(member)) >=
        plant.GetJointByName(member).position_lower_limits()(0));
  }

  // u limit
  for (int i = 0; i < N; i++) {
    auto ui = trajopt->input(i);
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  // toe position constraint in y direction (avoid leg crossing)
  auto left_foot_y_constraint = std::make_shared<PointPositionConstraint<double>>(
      plant, "toe_left", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
      0.05 * VectorXd::Ones(1), 0.6 * VectorXd::Ones(1), "toe_left_y");
  auto right_foot_y_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), Eigen::RowVector3d(0, 1, 0),
          -0.6 * VectorXd::Ones(1), -0.05 * VectorXd::Ones(1), "toe_right_y");
  // scaling
  if (FLAGS_is_scale_constraint) {
    std::unordered_map<int, double> odbp_constraint_scale;
    odbp_constraint_scale.insert(std::pair<int, double>(0, 0.5));
    left_foot_y_constraint->SetConstraintScaling(odbp_constraint_scale);
    right_foot_y_constraint->SetConstraintScaling(odbp_constraint_scale);
  }
  int start_idx = 0;
  for (int mode = 0; mode < 3; ++mode) {
    for (int index = 0; index < mode_lengths[mode] - 1; index++) {
      // Assumes mode_lengths are the same across modes
      cout << start_idx + index << endl;
      auto x_i = trajopt->state(start_idx + index);
      trajopt->AddConstraint(left_foot_y_constraint, x_i.head(n_q));
      trajopt->AddConstraint(right_foot_y_constraint, x_i.head(n_q));
    }
    start_idx += mode_lengths[mode] - 1;
  }

  // toe height constraint (avoid foot scuffing)
  Vector3d z_hat(0, 0, 1);
  Eigen::Quaterniond q;
  q.setFromTwoVectors(z_hat, ground_normal);
  Eigen::Matrix3d T_ground_incline = q.matrix().transpose();
  auto right_foot_constraint_z =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), T_ground_incline.row(2),
          0.08 * VectorXd::Ones(1),
          std::numeric_limits<double>::infinity() * VectorXd::Ones(1),
          "toe_right_z");
  auto x_mid = trajopt->state(N / 2);
  trajopt->AddConstraint(right_foot_constraint_z, x_mid.head(n_q));

  // Optional -- constraint on initial floating base
  trajopt->AddConstraint(x0(0) == 1);
  // Optional -- constraint on the forces magnitude
  /*for (unsigned int mode = 0; mode < timesteps.size(); mode++) {
    for (int index = 0; index < timesteps[mode]; index++) {
      auto lambda = trajopt->force(mode, index);
      trajopt->AddLinearConstraint(lambda(2) <= 700);
      trajopt->AddLinearConstraint(lambda(5) <= 700);
      trajopt->AddLinearConstraint(lambda(6) >= -1000);  // left leg four bar
      trajopt->AddLinearConstraint(lambda(6) <= 1000);   // left leg four bar
      trajopt->AddLinearConstraint(lambda(7) >= -500);   // right leg four bar
      trajopt->AddLinearConstraint(lambda(7) <= 500);    // right leg four bar
    }
  }*/
  /*for (int i = 0; i < N - 1; i++) {
    auto lambda = trajopt->collocation_force(0, i);
    trajopt->AddLinearConstraint(lambda(2) <= 700);
    trajopt->AddLinearConstraint(lambda(5) <= 700);
    trajopt->AddLinearConstraint(lambda(6) >= -1000);  // left leg four bar
    trajopt->AddLinearConstraint(lambda(6) <= 1000);   // left leg four bar
    trajopt->AddLinearConstraint(lambda(7) >= -500);   // right leg four bar
    trajopt->AddLinearConstraint(lambda(7) <= 500);    // right leg four bar
  }*/
  // Optional -- constraint on normal force
  for (unsigned int mode = 0; mode < timesteps.size(); mode++) {
    for (int index = 0; index < timesteps[mode]; index++) {
      auto lambda = trajopt->force(mode, index);
      trajopt->AddLinearConstraint(lambda(2) >= 10);
      trajopt->AddLinearConstraint(lambda(5) >= 10);
    }
  }
  // Optional -- constraint on u
  /*for (int i = 0; i < N; i++) {
    auto ui = trajopt->input(i);
    trajopt->AddLinearConstraint(ui(6) >= 0);
  }*/
  // Optional -- constraint left four-bar force (seems to help in high speed)
  if (constrain_stance_leg_fourbar_force) {
    for (unsigned int mode = 0; mode < timesteps.size(); mode++) {
      for (int index = 0; index < timesteps[mode]; index++) {
        auto lambda = trajopt->force(mode, index);
        trajopt->AddLinearConstraint(lambda(6) <= 0);  // left leg four bar
      }
    }
  }

  // Scale decision variable
  double s_q_toe = 1;
  double s_v_toe_l = 1;
  double s_v_toe_r = 1;
  if (FLAGS_is_scale_variable) {
    std::vector<int> idx_list;
    // time
    trajopt->ScaleTimeVariables(0.008);
    // state
    trajopt->ScaleStateVariables({0, 1, 2, 3}, 0.5);
    if (s_q_toe > 1) {
      trajopt->ScaleStateVariables({n_q - 2, n_q - 1}, s_q_toe);
    }
    idx_list.clear();
    for (int i = n_q; i < n_q + n_v - 2; i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleStateVariables(idx_list, 10);
    trajopt->ScaleStateVariable(n_q + n_v - 2, 10 * s_v_toe_l);
    trajopt->ScaleStateVariable(n_q + n_v - 1, 10 * s_v_toe_r);
    // input
    trajopt->ScaleInputVariables({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, 100);
    // force
    idx_list.clear();
    for (int i = 0; i < ls_dataset.countConstraintsWithoutSkipping(); i++) {
      idx_list.push_back(i);
    }
    trajopt->ScaleForceVariables(0, idx_list, 1000);
    idx_list.clear();
    for (int i = 0; i < rs_dataset.countConstraintsWithoutSkipping(); i++) {
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

  // add cost
  MatrixXd W_Q = w_Q * MatrixXd::Identity(n_v, n_v);
  MatrixXd W_R = w_R * MatrixXd::Identity(n_u, n_u);
  W_Q(n_v - 2, n_v - 2) /= (s_v_toe_l * s_v_toe_l);
  W_Q(n_v - 1, n_v - 1) /= (s_v_toe_r * s_v_toe_r);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * W_Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * W_R * u);

  // add cost on force difference wrt time
  // TODO(yangwill) fix indexing
  //  bool diff_with_force_at_collocation = false;
  //  if (w_lambda_diff) {
  //    for (int i = 0; i < N - 1; i++) {
  //      auto lambda0 = trajopt->force(0, i);
  //      auto lambda1 = trajopt->force(0, i + 1);
  //      auto lambdac = trajopt->collocation_force(0, i);
  //      if (diff_with_force_at_collocation) {
  //        trajopt->AddCost(w_lambda_diff *
  //                         (lambda0 - lambdac).dot(lambda0 - lambdac));
  //        trajopt->AddCost(w_lambda_diff *
  //                         (lambdac - lambda1).dot(lambdac - lambda1));
  //      } else {
  //        trajopt->AddCost(w_lambda_diff *
  //                         (lambda0 - lambda1).dot(lambda0 - lambda1));
  //      }
  //    }
  //  }
  // add cost on vel difference wrt time
  MatrixXd Q_v_diff = w_v_diff * MatrixXd::Identity(n_v, n_v);
  Q_v_diff(n_v - 2, n_v - 2) /= (s_v_toe_l * s_v_toe_l);
  Q_v_diff(n_v - 1, n_v - 1) /= (s_v_toe_r * s_v_toe_r);
  if (w_v_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto v0 = trajopt->state(i).tail(n_v);
      auto v1 = trajopt->state(i + 1).tail(n_v);
      trajopt->AddCost((v0 - v1).dot(Q_v_diff * (v0 - v1)));
    }
  }
  // add cost on input difference wrt time
  if (w_u_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto u_i = trajopt->input(i);
      auto u_ip1 = trajopt->input(i + 1);
      trajopt->AddCost(w_u_diff * (u_i - u_ip1).dot(u_i - u_ip1));
    }
  }
  // add cost on joint position
  if (w_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q_i = trajopt->state(i).segment(7, 2);
      trajopt->AddCost(w_q_hip_roll * q_i.transpose() * q_i);
    }
  }
  if (w_q_hip_yaw) {
    for (int i = 0; i < N; i++) {
      auto q_i = trajopt->state(i).segment(9, 2);
      trajopt->AddCost(w_q_hip_yaw * q_i.transpose() * q_i);
    }
  }
  if (w_q_quat_xyz) {
    for (int i = 0; i < N; i++) {
      auto q_i = trajopt->state(i).segment(1, 3);
      trajopt->AddCost(w_q_quat_xyz * q_i.transpose() * q_i);
    }
  }

  // initial guess

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    MatrixXd decisionVars =
        loadSavedDecisionVars(FLAGS_data_directory + FLAGS_load_filename);
    trajopt->SetInitialGuessForAllVariables(decisionVars);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    trajopt->SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt->decision_variables().size()));

    // Do inverse kinematics to get q initial guess
    vector<VectorXd> q_seed =
        GetInitGuessForQ(N, FLAGS_stride_length, FLAGS_ground_incline, plant,
                         FLAGS_visualize_init_guess);
    // Do finite differencing to get v initial guess
    vector<VectorXd> v_seed =
        GetInitGuessForV(q_seed, FLAGS_duration / (N - 1), plant);
    for (int i = 0; i < N; i++) {
      auto xi = trajopt->state(i);
      VectorXd xi_seed(n_q + n_v);
      xi_seed << q_seed.at(i), v_seed.at(i);
      trajopt->SetInitialGuess(xi, xi_seed);
    }

    // Add more initial guess
    // A better initial guess is necessary if we don't restrict the
    // FLAGS_duration
    if (!FLAGS_fix_time) {
      // initial guess for input
      // These guesses are from a good solution
      for (int i = 0; i < N; i++) {
        auto u_i = trajopt->input(i);
        trajopt->SetInitialGuess(u_i(0), 20);
        trajopt->SetInitialGuess(u_i(2), -30);
        trajopt->SetInitialGuess(u_i(4), 30);
        trajopt->SetInitialGuess(u_i(6), 50);
        trajopt->SetInitialGuess(u_i(8), 20);
      }
      // initial guess for force (also forces at collocation)
      for (int i = 0; i < N; i++) {
        auto lambda = trajopt->force(0, i);
        trajopt->SetInitialGuess(lambda(2), 170);
        trajopt->SetInitialGuess(lambda(5), 170);
        trajopt->SetInitialGuess(lambda(6), -500);
        trajopt->SetInitialGuess(lambda(7), 50);
      }
      for (int i = 0; i < N - 1; i++) {
        auto lambda0 = trajopt->GetInitialGuess(trajopt->force(0, i));
        auto lambda1 = trajopt->GetInitialGuess(trajopt->force(0, i + 1));
        auto collocation_lambda = trajopt->collocation_force(0, i);
        trajopt->SetInitialGuess(collocation_lambda, (lambda0 + lambda1) / 2);
      }
      // initial guess for slack
      auto vars_kinematics = trajopt->collocation_slack_vars(0);
      for (int i = 0; i < vars_kinematics.size(); i++) {
        trajopt->SetInitialGuess(vars_kinematics(i), 0);
      }
      auto vars_quaternion = trajopt->quaternion_slack_vars(0);
      for (int i = 0; i < vars_quaternion.size(); i++) {
        trajopt->SetInitialGuess(vars_quaternion(i), 0);
      }
      // initial condition for timestep
      for (int i = 0; i < N - 1; i++) {
        auto h_var = trajopt->timestep(i);
        trajopt->SetInitialGuess(h_var(0), FLAGS_duration / (N - 1));
      }
      // initial condition for post impact velocity
      auto vp_var = trajopt->v_post_impact_vars_by_mode(0);
      trajopt->SetInitialGuess(
          vp_var(vel_map.at("base_wx")),
          trajopt->GetInitialGuess(x0(n_q + vel_map.at("base_wx"))));
      trajopt->SetInitialGuess(
          vp_var(vel_map.at("base_wy")),
          -trajopt->GetInitialGuess(x0(n_q + vel_map.at("base_wy"))));
      trajopt->SetInitialGuess(
          vp_var(vel_map.at("base_wz")),
          trajopt->GetInitialGuess(x0(n_q + vel_map.at("base_wz"))));
      trajopt->SetInitialGuess(
          vp_var(vel_map.at("base_vx")),
          trajopt->GetInitialGuess(x0(n_q + vel_map.at("base_vx"))));
      trajopt->SetInitialGuess(
          vp_var(vel_map.at("base_vy")),
          -trajopt->GetInitialGuess(x0(n_q + vel_map.at("base_vy"))));
      trajopt->SetInitialGuess(
          vp_var(vel_map.at("base_vz")),
          trajopt->GetInitialGuess(x0(n_q + vel_map.at("base_vz"))));
      for (auto l_r_pair : l_r_pairs) {
        for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
          // velocities
          trajopt->SetInitialGuess(
              vp_var(vel_map.at(asy_joint_names[i] + l_r_pair.second + "dot")),
              -trajopt->GetInitialGuess(
                  x0(n_q +
                     vel_map.at(asy_joint_names[i] + l_r_pair.first + "dot"))));
        }
        for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
          // velocities
          trajopt->SetInitialGuess(
              vp_var(vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")),
              trajopt->GetInitialGuess(
                  x0(n_q +
                     vel_map.at(sym_joint_names[i] + l_r_pair.first + "dot"))));
        }
      }  // end for (l_r_pairs)
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

  trajopt->CreateVisualizationCallback(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", 5);

  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(*trajopt).name() << endl;

  cout << "Solving DIRCON\n\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "Solve result: " << result.get_solution_result() << std::endl;

  const PiecewisePolynomial<double>& state_traj =
      trajopt->ReconstructStateTrajectory(result);
  const PiecewisePolynomial<double>& input_traj =
      trajopt->ReconstructInputTrajectory(result);

  LcmTrajectory::Trajectory decision_vars;
  decision_vars.traj_name = "decision_vars";
  decision_vars.datapoints = result.GetSolution();
  decision_vars.time_vector = VectorXd::Zero(decision_vars.datapoints.size());
  decision_vars.datatypes = vector<string>(decision_vars.datapoints.size());

  int num_modes = 3;
  std::vector<LcmTrajectory::Trajectory> trajectories;
  std::vector<std::string> trajectory_names;

  start_idx = 0;
  for (int mode = 0; mode < num_modes; ++mode) {
    LcmTrajectory::Trajectory traj_block;
    traj_block.traj_name = "state_input_trajectory" + std::to_string(mode);
    std::vector<double> breaks_copy =
        std::vector(state_traj.get_segment_times());
    traj_block.time_vector =
        Eigen::Map<Eigen::VectorXd>(breaks_copy.data(), breaks_copy.size())
            .segment(start_idx, timesteps[mode]);
    start_idx += timesteps[mode];
    traj_block.datapoints = generateStateAndInputMatrix(state_traj, input_traj,
                                                        traj_block.time_vector);
    // To store x and xdot at the knot points
    const vector<string>& state_datatypes =
        multibody::createStateNameVectorFromMap(plant);
    const vector<string>& input_datatypes =
        multibody::createActuatorNameVectorFromMap(plant);
    traj_block.datatypes.reserve(state_datatypes.size() +
                                 state_datatypes.size() +
                                 input_datatypes.size());
    traj_block.datatypes.insert(traj_block.datatypes.end(),
                                state_datatypes.begin(), state_datatypes.end());
    traj_block.datatypes.insert(traj_block.datatypes.end(),
                                state_datatypes.begin(), state_datatypes.end());
    traj_block.datatypes.insert(traj_block.datatypes.end(),
                                input_datatypes.begin(), input_datatypes.end());
    trajectories.push_back(traj_block);
    trajectory_names.push_back(traj_block.traj_name);
  }

  trajectories.push_back(decision_vars);
  trajectory_names.push_back(decision_vars.traj_name);
  LcmTrajectory saved_traj(trajectories, trajectory_names, "walking_trajectory",
                           "Decision variables and state/input trajectories "
                           "for walking");
  saved_traj.writeToFile(FLAGS_data_directory + FLAGS_save_filename);

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

// Do inverse kinematics to get configuration guess
vector<VectorXd> GetInitGuessForQ(int N, double stride_length,
                                  double ground_incline,
                                  const MultibodyPlant<double>& plant,
                                  bool visualize_init_guess) {
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

  for (int i = 0; i < N; i++) {
    double eps = 1e-3;
    Vector3d eps_vec = eps * VectorXd::Ones(3);
    Vector3d pelvis_pos(stride_length * i / (N - 1), 0.0, 1.0);
    double stance_toe_pos_x = stride_length / 2;
    Vector3d stance_toe_pos(stance_toe_pos_x, 0.12,
                            0.05 + tan(-ground_incline) * stance_toe_pos_x);
    double swing_toe_pos_x =
        -stride_length / 2 + 2 * stride_length * i / (N - 1);
    Vector3d swing_toe_pos(swing_toe_pos_x, -0.12,
                           0.05 + 0.1 * (-abs((i - N / 2.0) / (N / 2.0)) + 1) +
                               tan(-ground_incline) * swing_toe_pos_x);

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
                             stance_toe_pos - eps_vec,
                             stance_toe_pos + eps_vec);
    ik.AddPositionConstraint(toe_right_frame, Vector3d(0, 0, 0), world_frame,
                             swing_toe_pos - eps_vec, swing_toe_pos + eps_vec);
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
    VectorXd q_sol_normd(n_q);
    q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
    q_ik_guess = q_sol_normd;
    q_init_guess.push_back(q_sol_normd);

    if (visualize_init_guess) {
      // Build temporary diagram for visualization
      drake::systems::DiagramBuilder<double> builder_ik;
      SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<SceneGraph>();
      scene_graph_ik.set_name("scene_graph_ik");
      MultibodyPlant<double> plant_ik(0.0);
      Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));
      multibody::addFlatTerrain(&plant_ik, &scene_graph_ik, .8, .8,
                                ground_normal);
      Parser parser(&plant_ik, &scene_graph_ik);
      string full_name =
          FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
      parser.AddModelFromFile(full_name);
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
      simulator.AdvanceTo(1.0 / N);
    }
  }

  return q_init_guess;
}

// Get v by finite differencing q
vector<VectorXd> GetInitGuessForV(const vector<VectorXd>& q_seed, double dt,
                                  const MultibodyPlant<double>& plant) {
  vector<VectorXd> qdot_seed;
  for (unsigned int i = 0; i < q_seed.size(); i++) {
    if (i == 0) {
      qdot_seed.emplace_back((q_seed[i + 1] - q_seed[i]) / dt);
    } else if (i == q_seed.size() - 1) {
      qdot_seed.push_back((q_seed[i] - q_seed[i - 1]) / dt);
    } else {
      VectorXd v_plus = (q_seed[i + 1] - q_seed[i]) / dt;
      VectorXd v_minus = (q_seed[i] - q_seed[i - 1]) / dt;
      qdot_seed.push_back((v_plus + v_minus) / 2);
    }
  }

  // Convert qdot to v
  vector<VectorXd> v_seed;
  for (unsigned int i = 0; i < q_seed.size(); i++) {
    auto context = plant.CreateDefaultContext();
    plant.SetPositions(context.get(), q_seed[i]);
    VectorXd v(plant.num_velocities());
    plant.MapQDotToVelocity(*context, qdot_seed[i], &v);
    v_seed.push_back(v);
  }
  return v_seed;
}

MatrixXd loadSavedDecisionVars(const string& filepath) {
  const LcmTrajectory& loaded_decision_vars = LcmTrajectory(filepath);
  return loaded_decision_vars.getTrajectory("decision_vars").datapoints;
}

MatrixXd generateStateAndInputMatrix(const PiecewisePolynomial<double>& states,
                                     const PiecewisePolynomial<double>& inputs,
                                     VectorXd times) {
  int num_states = states.value(0).size();
  int num_inputs = inputs.value(0).size();
  auto state_derivatives = states.MakeDerivative(1);
  MatrixXd states_matrix = MatrixXd::Zero(num_states, times.size());
  MatrixXd state_derivatives_matrix = MatrixXd::Zero(num_states, times.size());
  MatrixXd inputs_matrix = MatrixXd::Zero(num_inputs, times.size());

  for (int i = 0; i < times.size(); ++i) {
    states_matrix.col(i) = states.value(times[i]);
    state_derivatives_matrix.col(i) = state_derivatives->value(times[i]);
    if (times.size() == 1) {
      state_derivatives_matrix.col(i) = VectorXd::Zero(num_states);
    }
    inputs_matrix.col(i) = inputs.value(times[i]);
  }
  MatrixXd states_and_inputs(num_states + num_states + num_inputs,
                             times.size());
  states_and_inputs.topRows(num_states) = states_matrix;
  states_and_inputs.block(num_states, 0, num_states, times.size()) =
      state_derivatives_matrix;
  states_and_inputs.bottomRows(num_inputs) = inputs_matrix;

  return states_and_inputs;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}
