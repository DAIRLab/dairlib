#include <algorithm>
#include <iostream>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "drake/geometry/drake_visualizer.h"
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
using drake::geometry::DrakeVisualizer;
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

DEFINE_string(init_file, "", "the file name of initial guess");
DEFINE_string(data_directory, "../dairlib_data/cassie_trajopt_data/",
              "directory to save/read data");
DEFINE_string(save_filename, "default_filename",
              "Filename to save decision "
              "vars to.");
DEFINE_bool(store_data, false, "To store solution or not");

// SNOPT parameters
DEFINE_int32(max_iter, 100000, "Iteration limit");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");

// Gait and traj opt parameters
DEFINE_int32(n_node, 16, "Number of nodes");
DEFINE_bool(is_fix_time, true, "Whether to fix the duration of gait or not");
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

// Do inverse kinematics to get configuration guess
vector<VectorXd> GetInitGuessForQ(int N, double stride_length,
                                  double ground_incline,
                                  const MultibodyPlant<double>& plant,
                                  bool visualize_init_guess = false) {
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  map<string, int> positions_map = multibody::MakeNameToPositionsMap(plant);

  vector<VectorXd> q_init_guess;
  VectorXd q_ik_guess = VectorXd::Zero(n_q);

  map<string, double> pos_value_map;
  Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
  quat.normalize();
  pos_value_map["base_qw"] = quat(0);
  pos_value_map["base_qx"] = quat(1);
  pos_value_map["base_qy"] = quat(2);
  pos_value_map["base_qz"] = quat(3);
  pos_value_map["base_x"] = 0.000889849;
  pos_value_map["base_y"] = 0.000626865;
  pos_value_map["base_z"] = 1.0009;
  pos_value_map["hip_roll_left"] = -0.0112109;
  pos_value_map["hip_roll_right"] = 0.00927845;
  pos_value_map["hip_yaw_left"] = -0.000600725;
  pos_value_map["hip_yaw_right"] = -0.000895805;
  pos_value_map["hip_pitch_left"] = 1.15086;
  pos_value_map["hip_pitch_right"] = 0.610808;
  pos_value_map["knee_left"] = -1.38608;
  pos_value_map["knee_right"] = -1.35926;
  pos_value_map["ankle_joint_left"] = 0.806192;
  pos_value_map["ankle_joint_right"] = 1.00716;
  pos_value_map["toe_left"] = -M_PI / 2;
  pos_value_map["toe_right"] = -M_PI / 2;

  for (auto pair : pos_value_map) {
    q_ik_guess(positions_map.at(pair.first)) = pair.second;
  }

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
      multibody::AddFlatTerrain(&plant_ik, &scene_graph_ik, .8, .8,
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

      multibody::ConnectTrajectoryVisualizer(&plant_ik, &builder_ik,
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
      qdot_seed.push_back((q_seed[i + 1] - q_seed[i]) / dt);
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

void DoMain(double duration, double stride_length, double ground_incline,
            bool is_fix_time, int n_node, int max_iter,
            const string& data_directory, const string& init_file, double tol,
            bool to_store_data, int scale_option) {
  // Dircon parameter
  double minimum_timestep = 0.01;
  DRAKE_DEMAND(duration / (n_node - 1) >= minimum_timestep);
  // If the node density is too low, it's harder for SNOPT to converge well.
  double max_distance_per_node = 0.2 / 16;
  DRAKE_DEMAND((stride_length / n_node) <= max_distance_per_node);

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

  Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));
  multibody::AddFlatTerrain(&plant, &scene_graph, 1, 1, ground_normal);

  Parser parser(&plant, &scene_graph);

  string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  plant.Finalize();

  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

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

  // Left support
  vector<DirconKinematicData<double>*> ls_constraint;
  ls_constraint.push_back(&left_toe_front_constraint);
  ls_constraint.push_back(&left_toe_rear_constraint);
  ls_constraint.push_back(&distance_constraint_left);
  ls_constraint.push_back(&distance_constraint_right);
  auto ls_dataset = DirconKinematicDataSet<double>(plant, &ls_constraint,
                                                   skip_constraint_inds);
  // Right support
  vector<DirconKinematicData<double>*> rs_constraint;
  rs_constraint.push_back(&right_toe_front_constraint);
  rs_constraint.push_back(&right_toe_rear_constraint);
  rs_constraint.push_back(&distance_constraint_left);
  rs_constraint.push_back(&distance_constraint_right);
  auto rs_dataset = DirconKinematicDataSet<double>(plant, &rs_constraint,
                                                   skip_constraint_inds);

  // Set up options
  std::vector<DirconOptions> options_list;
  options_list.push_back(DirconOptions(ls_dataset.countConstraints(), plant));
  options_list.push_back(DirconOptions(rs_dataset.countConstraints(), plant));

  // set force cost weight
  for (int i = 0; i < 2; i++) {
    options_list[i].setForceCost(w_lambda);
  }

  // Be careful in setting relative constraint, because we skip constraints
  ///                 || lf/rf | lr/rr | fourbar
  /// Before skipping || 0 1 2 | 3 4 5 | 6 7
  /// After skipping  || 0 1 2 |   3 4 | 5 6
  for (int i = 0; i < 2; i++) {
    options_list[i].setConstraintRelative(0, true);
    options_list[i].setConstraintRelative(1, true);
    options_list[i].setConstraintRelative(3, true);
  }

  int base_qw_idx = pos_map.at("base_qw");
  int base_qx_idx = pos_map.at("base_qx");
  int base_qy_idx = pos_map.at("base_qy");
  int base_qz_idx = pos_map.at("base_qz");
  int base_x_idx = pos_map.at("base_x");
  int base_y_idx = pos_map.at("base_y");
  int base_z_idx = pos_map.at("base_z");
  int hip_roll_left_idx = pos_map.at("hip_roll_left");
  int hip_roll_right_idx = pos_map.at("hip_roll_right");
  int hip_yaw_left_idx = pos_map.at("hip_yaw_left");
  int hip_yaw_right_idx = pos_map.at("hip_yaw_right");
  int hip_pitch_left_idx = pos_map.at("hip_pitch_left");
  int hip_pitch_right_idx = pos_map.at("hip_pitch_right");
  int knee_left_idx = pos_map.at("knee_left");
  int knee_right_idx = pos_map.at("knee_right");
  int ankle_joint_left_idx = pos_map.at("ankle_joint_left");
  int ankle_joint_right_idx = pos_map.at("ankle_joint_right");
  int toe_left_idx = pos_map.at("toe_left");
  int toe_right_idx = pos_map.at("toe_right");

  int base_wx_idx = vel_map.at("base_wx");
  int base_wy_idx = vel_map.at("base_wy");
  int base_wz_idx = vel_map.at("base_wz");
  int base_vx_idx = vel_map.at("base_vx");
  int base_vy_idx = vel_map.at("base_vy");
  int base_vz_idx = vel_map.at("base_vz");
  int hip_roll_leftdot_idx = vel_map.at("hip_roll_leftdot");
  int hip_roll_rightdot_idx = vel_map.at("hip_roll_rightdot");
  int hip_yaw_leftdot_idx = vel_map.at("hip_yaw_leftdot");
  int hip_yaw_rightdot_idx = vel_map.at("hip_yaw_rightdot");
  int hip_pitch_leftdot_idx = vel_map.at("hip_pitch_leftdot");
  int hip_pitch_rightdot_idx = vel_map.at("hip_pitch_rightdot");
  int knee_leftdot_idx = vel_map.at("knee_leftdot");
  int knee_rightdot_idx = vel_map.at("knee_rightdot");
  int ankle_joint_leftdot_idx = vel_map.at("ankle_joint_leftdot");
  int ankle_joint_rightdot_idx = vel_map.at("ankle_joint_rightdot");
  int toe_leftdot_idx = vel_map.at("toe_leftdot");
  int toe_rightdot_idx = vel_map.at("toe_rightdot");

  int hip_roll_left_motor_idx = act_map.at("hip_roll_left_motor");
  int hip_roll_right_motor_idx = act_map.at("hip_roll_right_motor");
  int hip_yaw_left_motor_idx = act_map.at("hip_yaw_left_motor");
  int hip_yaw_right_motor_idx = act_map.at("hip_yaw_right_motor");
  int hip_pitch_left_motor_idx = act_map.at("hip_pitch_left_motor");
  int hip_pitch_right_motor_idx = act_map.at("hip_pitch_right_motor");
  int knee_left_motor_idx = act_map.at("knee_left_motor");
  int knee_right_motor_idx = act_map.at("knee_right_motor");
  int toe_left_motor_idx = act_map.at("toe_left_motor");
  int toe_right_motor_idx = act_map.at("toe_right_motor");

  // Constraint scaling
  if (FLAGS_is_scale_constraint) {
    for (int i = 0; i < 2; i++) {
      double s = 1;  // scale everything together
      // Dynamic constraints
      options_list[i].setDynConstraintScaling(
          {base_qw_idx, base_qx_idx, base_qy_idx, base_qz_idx}, s * 1.0 / 30.0);
      options_list[i].setDynConstraintScaling(
          {base_x_idx, base_y_idx, base_z_idx, hip_roll_left_idx,
           hip_roll_right_idx, hip_yaw_left_idx, hip_yaw_right_idx,
           hip_pitch_left_idx, hip_pitch_right_idx, knee_left_idx,
           knee_right_idx, ankle_joint_left_idx, ankle_joint_right_idx},
          s * 1.0 / 60.0);
      options_list[i].setDynConstraintScaling({toe_left_idx, toe_right_idx},
                                              s * 1.0 / 300.0);
      options_list[i].setDynConstraintScaling(
          {n_q + base_wx_idx, n_q + base_wy_idx, n_q + base_wz_idx,
           n_q + base_vx_idx, n_q + base_vy_idx, n_q + base_vz_idx,
           n_q + hip_roll_leftdot_idx, n_q + hip_roll_rightdot_idx,
           n_q + hip_yaw_leftdot_idx, n_q + hip_yaw_rightdot_idx},
          s * 1.0 / 600.0);
      options_list[i].setDynConstraintScaling(
          {n_q + hip_pitch_leftdot_idx, n_q + hip_pitch_rightdot_idx,
           n_q + knee_leftdot_idx, n_q + knee_rightdot_idx,
           n_q + ankle_joint_leftdot_idx, n_q + ankle_joint_rightdot_idx},
          s * 1.0 / 3000.0);
      options_list[i].setDynConstraintScaling(
          {n_q + toe_leftdot_idx, n_q + toe_rightdot_idx}, s * 1.0 / 60000.0);

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
      options_list[i].setImpConstraintScaling(
          {base_wx_idx, base_wy_idx, base_wz_idx}, s / 50.0);
      options_list[i].setImpConstraintScaling(
          {base_vx_idx, base_vy_idx, base_vz_idx}, s / 300.0);
      options_list[i].setImpConstraintScaling(
          {hip_roll_leftdot_idx, hip_roll_rightdot_idx}, s / 24.0);
      options_list[i].setImpConstraintScaling(
          {hip_yaw_leftdot_idx, hip_yaw_rightdot_idx}, s / 6.0);
      options_list[i].setImpConstraintScaling(
          {hip_pitch_leftdot_idx, hip_pitch_rightdot_idx, knee_leftdot_idx,
           knee_rightdot_idx},
          s / 12.0);
      options_list[i].setImpConstraintScaling(
          {ankle_joint_leftdot_idx, ankle_joint_rightdot_idx}, s / 2.0);
      options_list[i].setImpConstraintScaling(
          {toe_leftdot_idx, toe_rightdot_idx}, s);
    }
  }

  // timesteps and modes setting
  vector<double> min_dt;
  vector<double> max_dt;
  min_dt.push_back(minimum_timestep);
  min_dt.push_back(minimum_timestep);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  vector<int> num_time_samples;
  num_time_samples.push_back(n_node);
  num_time_samples.push_back(1);
  vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&ls_dataset);
  dataset_list.push_back(&rs_dataset);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, num_time_samples, min_dt, max_dt, dataset_list, options_list);
  auto& prog = trajopt->prog();

  // Snopt settings
  //  prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
  //                           "../snopt.out");
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                       "Major iterations limit", max_iter);
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                       "Iterations limit", 100000);  // QP subproblems
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);
  prog.SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Scale option",
      scale_option);  // snopt doc said try 2 if seeing snopta exit 40
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                       "Major optimality tolerance",
                       tol);  // target nonlinear constraint violation
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                       "Major feasibility tolerance",
                       tol);  // target complementarity gap

  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++) N += num_time_samples[i];
  N -= num_time_samples.size() - 1;  // because of overlaps between modes

  // Get the decision variables that will be used
  auto u = trajopt->input();
  auto x = trajopt->state();
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);

  // Fix time duration
  if (is_fix_time) {
    trajopt->AddDurationBounds(duration, duration);
  }

  // x position constraint
  prog.AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  prog.AddBoundingBoxConstraint(stride_length, stride_length,
                                    xf(pos_map.at("base_x")));

  // height constraint
  //  prog.AddLinearConstraint(x0(pos_map.at("base_z")) == 1);
  //  prog.AddLinearConstraint(xf(pos_map.at("base_z")) == 1.1);

  // initial pelvis position
  // prog.AddLinearConstraint(x0(pos_map.at("base_y")) == 0);

  // pelvis pose constraints
  //  prog.AddConstraintToAllKnotPoints(x(pos_map.at("base_qw")) ==
  //  1); prog.AddConstraintToAllKnotPoints(x(pos_map.at("base_qx"))
  //  == 0);
  //  prog.AddConstraintToAllKnotPoints(x(pos_map.at("base_qy")) ==
  //  0); prog.AddConstraintToAllKnotPoints(x(pos_map.at("base_qz"))
  //  == 0);

  // start/end velocity constraints
  //  prog.AddLinearConstraint(x0.tail(n_v) == VectorXd::Zero(n_v));
  //  prog.AddLinearConstraint(xf.tail(n_v) == VectorXd::Zero(n_v));

  // Floating base periodicity
  prog.AddLinearConstraint(x0(pos_map.at("base_qw")) ==
                               xf(pos_map.at("base_qw")));
  prog.AddLinearConstraint(x0(pos_map.at("base_qx")) ==
                               -xf(pos_map.at("base_qx")));
  prog.AddLinearConstraint(x0(pos_map.at("base_qy")) ==
                               xf(pos_map.at("base_qy")));
  prog.AddLinearConstraint(x0(pos_map.at("base_qz")) ==
                               -xf(pos_map.at("base_qz")));
  prog.AddLinearConstraint(x0(pos_map.at("base_y")) ==
                               -xf(pos_map.at("base_y")));
  if (ground_incline == 0) {
    prog.AddLinearConstraint(x0(pos_map.at("base_z")) ==
                                 xf(pos_map.at("base_z")));
  }
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_wx")) ==
                               xf(n_q + vel_map.at("base_wx")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_wy")) ==
                               -xf(n_q + vel_map.at("base_wy")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_wz")) ==
                               xf(n_q + vel_map.at("base_wz")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_vx")) ==
                               xf(n_q + vel_map.at("base_vx")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_vy")) ==
                               -xf(n_q + vel_map.at("base_vy")));
  prog.AddLinearConstraint(x0(n_q + vel_map.at("base_vz")) ==
                               xf(n_q + vel_map.at("base_vz")));

  // The legs joint positions/velocities/torque should be mirrored between legs
  // (notice that hip yaw and roll should be asymmetric instead of symmetric.)
  for (const auto& l_r_pair : l_r_pairs) {
    for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
      // positions
      prog.AddLinearConstraint(
          x0(pos_map.at(asy_joint_names[i] + l_r_pair.first)) ==
          -xf(pos_map.at(asy_joint_names[i] + l_r_pair.second)));
      // velocities
      prog.AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_names[i] + l_r_pair.first + "dot")) ==
          -xf(n_q + vel_map.at(asy_joint_names[i] + l_r_pair.second + "dot")));
      // inputs
      prog.AddLinearConstraint(
          u0(act_map.at(asy_joint_names[i] + l_r_pair.first + "_motor")) ==
          -uf(act_map.at(asy_joint_names[i] + l_r_pair.second + "_motor")));
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      // positions
      prog.AddLinearConstraint(
          x0(pos_map.at(sym_joint_names[i] + l_r_pair.first)) ==
          xf(pos_map.at(sym_joint_names[i] + l_r_pair.second)));
      // velocities
      prog.AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")));
      // inputs (ankle joint is not actuated)
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        prog.AddLinearConstraint(
            u0(act_map.at(sym_joint_names[i] + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_names[i] + l_r_pair.second + "_motor")));
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
    prog.AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                             VectorXd::Constant(n_u, +300),
                                             ui);
  }

  // toe position constraint in y direction (avoid leg crossing)
  auto left_foot_constraint = std::make_shared<PointPositionConstraint<double>>(
      plant, "toe_left", Vector3d::Zero(), MatrixXd::Identity(3, 3).row(1),
      0.05 * VectorXd::Ones(1),
      std::numeric_limits<double>::infinity() * VectorXd::Ones(1),
      "toe_left_y");
  auto right_foot_constraint =
      std::make_shared<PointPositionConstraint<double>>(
          plant, "toe_right", Vector3d::Zero(), MatrixXd::Identity(3, 3).row(1),
          -std::numeric_limits<double>::infinity() * VectorXd::Ones(1),
          -0.05 * VectorXd::Ones(1), "toe_right_y");
  // scaling
  if (FLAGS_is_scale_constraint) {
    std::unordered_map<int, double> odbp_constraint_scale;
    odbp_constraint_scale.insert(std::pair<int, double>(0, 0.5));
    left_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
    right_foot_constraint->SetConstraintScaling(odbp_constraint_scale);
  }
  for (int index = 0; index < num_time_samples[0]; index++) {
    auto x = trajopt->state(index);
    prog.AddConstraint(left_foot_constraint, x.head(n_q));
    prog.AddConstraint(right_foot_constraint, x.head(n_q));
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
  auto x_mid = trajopt->state(num_time_samples[0] / 2);
  prog.AddConstraint(right_foot_constraint_z, x_mid.head(n_q));

  // Optional -- constraint on initial floating base
  prog.AddConstraint(x0(pos_map.at("base_qw")) == 1);
  // Optional -- constraint on the forces magnitude
  /*for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
    for (int index = 0; index < num_time_samples[mode]; index++) {
      auto lambda = trajopt->force(mode, index);
      prog.AddLinearConstraint(lambda(2) <= 700);
      prog.AddLinearConstraint(lambda(5) <= 700);
      prog.AddLinearConstraint(lambda(6) >= -1000);  // left leg four bar
      prog.AddLinearConstraint(lambda(6) <= 1000);   // left leg four bar
      prog.AddLinearConstraint(lambda(7) >= -500);   // right leg four bar
      prog.AddLinearConstraint(lambda(7) <= 500);    // right leg four bar
    }
  }*/
  /*for (int i = 0; i < N - 1; i++) {
    auto lambda = trajopt->collocation_force(0, i);
    prog.AddLinearConstraint(lambda(2) <= 700);
    prog.AddLinearConstraint(lambda(5) <= 700);
    prog.AddLinearConstraint(lambda(6) >= -1000);  // left leg four bar
    prog.AddLinearConstraint(lambda(6) <= 1000);   // left leg four bar
    prog.AddLinearConstraint(lambda(7) >= -500);   // right leg four bar
    prog.AddLinearConstraint(lambda(7) <= 500);    // right leg four bar
  }*/
  // Optional -- constraint on normal force
  for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
    for (int index = 0; index < num_time_samples[mode]; index++) {
      auto lambda = trajopt->force(mode, index);
      prog.AddLinearConstraint(lambda(2) >= 10);
      prog.AddLinearConstraint(lambda(5) >= 10);
    }
  }
  // Optional -- constraint on u
  /*for (int i = 0; i < N; i++) {
    auto ui = trajopt->input(i);
    prog.AddLinearConstraint(ui(6) >= 0);
  }*/
  // Optional -- constraint left four-bar force (seems to help in high speed)
  if (constrain_stance_leg_fourbar_force) {
    for (unsigned int mode = 0; mode < num_time_samples.size(); mode++) {
      for (int index = 0; index < num_time_samples[mode]; index++) {
        auto lambda = trajopt->force(mode, index);
        prog.AddLinearConstraint(lambda(6) <= 0);  // left leg four bar
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
    trajopt->ScaleStateVariables(
        {base_qw_idx, base_qx_idx, base_qy_idx, base_qz_idx}, 0.5);
    if (s_q_toe > 1) {
      trajopt->ScaleStateVariables({toe_left_idx, toe_right_idx}, s_q_toe);
    }
    std::set<int> idx_set;
    for (int i = n_q; i < n_q + n_v; i++) {
      idx_set.insert(i);
    }
    idx_set.erase(n_q + vel_map.at("toe_leftdot"));
    idx_set.erase(n_q + vel_map.at("toe_rightdot"));
    idx_list.clear();
    idx_list.assign(idx_set.begin(), idx_set.end());
    trajopt->ScaleStateVariables(idx_list, 10);
    trajopt->ScaleStateVariable(n_q + vel_map.at("toe_leftdot"),
                                10 * s_v_toe_l);
    trajopt->ScaleStateVariable(n_q + vel_map.at("toe_rightdot"),
                                10 * s_v_toe_r);
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

  W_Q(vel_map.at("toe_leftdot"), vel_map.at("toe_leftdot")) /=
      (s_v_toe_l * s_v_toe_l);
  W_Q(vel_map.at("toe_rightdot"), vel_map.at("toe_rightdot")) /=
      (s_v_toe_r * s_v_toe_r);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * W_Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * W_R * u);

  // add cost on force difference wrt time
  bool diff_with_force_at_collocation = false;
  if (w_lambda_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto lambda0 = trajopt->force(0, i);
      auto lambda1 = trajopt->force(0, i + 1);
      auto lambdac = trajopt->collocation_force(0, i);
      if (diff_with_force_at_collocation) {
        prog.AddCost(w_lambda_diff *
                                (lambda0 - lambdac).dot(lambda0 - lambdac));
        prog.AddCost(w_lambda_diff *
                                (lambdac - lambda1).dot(lambdac - lambda1));
      } else {
        prog.AddCost(w_lambda_diff *
                                (lambda0 - lambda1).dot(lambda0 - lambda1));
      }
    }
  }
  // add cost on vel difference wrt time
  MatrixXd Q_v_diff = w_v_diff * MatrixXd::Identity(n_v, n_v);
  Q_v_diff(vel_map.at("toe_leftdot"), vel_map.at("toe_leftdot")) /=
      (s_v_toe_l * s_v_toe_l);
  Q_v_diff(vel_map.at("toe_rightdot"), vel_map.at("toe_rightdot")) /=
      (s_v_toe_r * s_v_toe_r);
  if (w_v_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto v0 = trajopt->state(i).tail(n_v);
      auto v1 = trajopt->state(i + 1).tail(n_v);
      prog.AddCost((v0 - v1).dot(Q_v_diff * (v0 - v1)));
    }
  }
  // add cost on input difference wrt time
  if (w_u_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto u0 = trajopt->input(i);
      auto u1 = trajopt->input(i + 1);
      prog.AddCost(w_u_diff * (u0 - u1).dot(u0 - u1));
    }
  }
  // add cost on joint position
  if (w_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q1 = trajopt->state(i).segment<1>(hip_roll_left_idx);
      auto q2 = trajopt->state(i).segment<1>(hip_roll_right_idx);
      prog.AddCost(w_q_hip_yaw * q1.transpose() * q1);
      prog.AddCost(w_q_hip_yaw * q2.transpose() * q2);
    }
  }
  if (w_q_hip_yaw) {
    for (int i = 0; i < N; i++) {
      auto q1 = trajopt->state(i).segment<1>(hip_yaw_left_idx);
      auto q2 = trajopt->state(i).segment<1>(hip_yaw_right_idx);
      prog.AddCost(w_q_hip_yaw * q1.transpose() * q1);
      prog.AddCost(w_q_hip_yaw * q2.transpose() * q2);
    }
  }
  if (w_q_quat_xyz) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(base_qx_idx, 3);
      prog.AddCost(w_q_quat_xyz * q.transpose() * q);
    }
  }

  // initial guess
  if (!init_file.empty()) {
    MatrixXd z0 = readCSV(data_directory + init_file);
    prog.SetInitialGuessForAllVariables(z0);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    prog.SetInitialGuessForAllVariables(
        VectorXd::Random(prog.decision_variables().size()));

    // Do inverse kinematics to get q initial guess
    vector<VectorXd> q_seed = GetInitGuessForQ(
        N, stride_length, ground_incline, plant, FLAGS_visualize_init_guess);
    // Do finite differencing to get v initial guess
    vector<VectorXd> v_seed =
        GetInitGuessForV(q_seed, duration / (N - 1), plant);
    for (int i = 0; i < N; i++) {
      auto xi = trajopt->state(i);
      VectorXd xi_seed(n_q + n_v);
      xi_seed << q_seed.at(i), v_seed.at(i);
      prog.SetInitialGuess(xi, xi_seed);
    }

    // Add more initial guess
    // A better initial guess is necessary if we don't restrict the duration
    if (!is_fix_time) {
      // initial guess for input
      // These guesses are from a good solution
      for (int i = 0; i < N; i++) {
        auto u = trajopt->input(i);
        prog.SetInitialGuess(u(hip_roll_left_motor_idx), 20);
        prog.SetInitialGuess(u(hip_yaw_left_motor_idx), -30);
        prog.SetInitialGuess(u(hip_pitch_left_motor_idx), 30);
        prog.SetInitialGuess(u(knee_left_motor_idx), 50);
        prog.SetInitialGuess(u(toe_left_motor_idx), 20);
      }
      // initial guess for force (also forces at collocation)
      for (int i = 0; i < N; i++) {
        auto lambda = trajopt->force(0, i);
        prog.SetInitialGuess(lambda(2), 170);
        prog.SetInitialGuess(lambda(5), 170);
        prog.SetInitialGuess(lambda(6), -500);
        prog.SetInitialGuess(lambda(7), 50);
      }
      for (int i = 0; i < N - 1; i++) {
        auto lambda0 = prog.GetInitialGuess(trajopt->force(0, i));
        auto lambda1 = prog.GetInitialGuess(trajopt->force(0, i + 1));
        auto collocation_lambda = trajopt->collocation_force(0, i);
        prog.SetInitialGuess(collocation_lambda, (lambda0 + lambda1) / 2);
      }
      // initial guess for slack
      auto vars_kinematics = trajopt->collocation_slack_vars(0);
      for (int i = 0; i < vars_kinematics.size(); i++) {
        prog.SetInitialGuess(vars_kinematics(i), 0);
      }
      auto vars_quaternion = trajopt->quaternion_slack_vars(0);
      for (int i = 0; i < vars_quaternion.size(); i++) {
        prog.SetInitialGuess(vars_quaternion(i), 0);
      }
      // initial condition for timestep
      for (int i = 0; i < N - 1; i++) {
        auto h_var = trajopt->time_step(i);
        prog.SetInitialGuess(h_var(0), duration / (N - 1));
      }
      // initial condition for post impact velocity
      auto vp_var = trajopt->v_post_impact_vars_by_mode(0);
      prog.SetInitialGuess(
          vp_var(vel_map.at("base_wx")),
          prog.GetInitialGuess(x0(n_q + vel_map.at("base_wx"))));
      prog.SetInitialGuess(
          vp_var(vel_map.at("base_wy")),
          -prog.GetInitialGuess(x0(n_q + vel_map.at("base_wy"))));
      prog.SetInitialGuess(
          vp_var(vel_map.at("base_wz")),
          prog.GetInitialGuess(x0(n_q + vel_map.at("base_wz"))));
      prog.SetInitialGuess(
          vp_var(vel_map.at("base_vx")),
          prog.GetInitialGuess(x0(n_q + vel_map.at("base_vx"))));
      prog.SetInitialGuess(
          vp_var(vel_map.at("base_vy")),
          -prog.GetInitialGuess(x0(n_q + vel_map.at("base_vy"))));
      prog.SetInitialGuess(
          vp_var(vel_map.at("base_vz")),
          prog.GetInitialGuess(x0(n_q + vel_map.at("base_vz"))));
      for (auto l_r_pair : l_r_pairs) {
        for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
          // velocities
          prog.SetInitialGuess(
              vp_var(vel_map.at(asy_joint_names[i] + l_r_pair.second + "dot")),
              -prog.GetInitialGuess(
                  x0(n_q +
                     vel_map.at(asy_joint_names[i] + l_r_pair.first + "dot"))));
        }
        for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
          // velocities
          prog.SetInitialGuess(
              vp_var(vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")),
              prog.GetInitialGuess(
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
    if ((prog.GetInitialGuess(xi.segment<4>(base_qw_idx)).norm() ==
        0) || std::isnan(prog.GetInitialGuess(
            xi.segment<4>(base_qw_idx)).norm())) {
      prog.SetInitialGuess(xi(pos_map.at("base_qw")), 1);
      prog.SetInitialGuess(xi(pos_map.at("base_qx")), 0);
      prog.SetInitialGuess(xi(pos_map.at("base_qy")), 0);
      prog.SetInitialGuess(xi(pos_map.at("base_qz")), 0);
    }
  }

  // Print out the scaling factor
  /*for (int i = 0; i < trajopt->decision_variables().size(); i++) {
    cout << trajopt->decision_variable(i) << ", ";
    cout << trajopt->decision_variable(i).get_id() << ", ";
    cout << trajopt->FindDecisionVariableIndex(trajopt->decision_variable(i))
         << ", ";
    auto scale_map = trajopt->GetVariableScaling();
    auto it = scale_map.find(i);
    if (it != scale_map.end()) {
      cout << it->second;
    } else {
      cout << "none";
    }
    cout << ", ";
    cout << prog.GetInitialGuess(trajopt->decision_variable(i));
    cout << endl;
  }*/

  double alpha = .2;
  int num_poses = 5;
  trajopt->CreateVisualizationCallback(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", num_poses, alpha);

  cout << "\nChoose the best solver: "
       << drake::solvers::ChooseBestSolver(prog).name() << endl;

  cout << "Solving DIRCON\n\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(prog, prog.initial_guess());
  SolutionResult solution_result = result.get_solution_result();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  // trajopt->PrintSolution();
  for (int i = 0; i < 100; i++) {
    cout << '\a';
  }  // making noise to notify

  // Testing - check if the nonlinear constraints are all satisfied
  // bool constraint_satisfied = solvers::CheckGenericConstraints(*trajopt,
  //                             result, tol);
  // cout << "constraint_satisfied = " << constraint_satisfied << endl;

  // store the solution of the decision variable
  VectorXd z = result.GetSolution(prog.decision_variables());
  if (to_store_data) {
    writeCSV(data_directory + string("z.csv"), z);
  }

  // Print the solution
  /*for (int i = 0; i < z.size(); i++) {
    cout << i << ": " << prog.decision_variables()[i] << ", " << z[i]
         << endl;
  }
  cout << endl;*/

  // store the time, state, and input at knot points
  VectorXd time_at_knots = trajopt->GetSampleTimes(result);
  MatrixXd state_at_knots = trajopt->GetStateSamples(result);
  MatrixXd input_at_knots = trajopt->GetInputSamples(result);
  //  state_at_knots.col(N - 1) = result.GetSolution(xf);
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
  if (to_store_data) {
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
  }

  // Print weight
  cout << "\nw_Q = " << w_Q << endl;
  cout << "w_R = " << w_R << endl;
  cout << "w_lambda = " << w_lambda << endl;
  cout << "w_lambda_diff = " << w_lambda_diff << endl;
  cout << "w_v_diff = " << w_v_diff << endl;
  cout << "w_u_diff = " << w_u_diff << endl;
  cout << "w_q_hip_roll = " << w_q_hip_roll << endl;
  cout << "w_q_hip_yaw = " << w_q_hip_yaw << endl;
  cout << "w_q_quat_xyz = " << w_q_quat_xyz << endl;

  // Print the result
  cout << "\n" << to_string(solution_result) << endl;
  cout << "Solve time:" << elapsed.count() << std::endl;
  cout << "Cost:" << result.get_optimal_cost() << std::endl;
  // Check which solver was used
  cout << "Solver: " << result.get_solver_id().name() << "\n\n";

  // Calculate each term of the cost
  double total_cost = 0;
  double cost_x = 0;
  for (int i = 0; i < N - 1; i++) {
    auto v0 = state_at_knots.col(i).tail(n_v);
    auto v1 = state_at_knots.col(i + 1).tail(n_v);
    auto h = time_at_knots(i + 1) - time_at_knots(i);
    cost_x += ((v0.transpose() * W_Q * v0) * h / 2)(0);
    cost_x += ((v1.transpose() * W_Q * v1) * h / 2)(0);
  }
  total_cost += cost_x;
  cout << "cost_x = " << cost_x << endl;
  double cost_u = 0;
  for (int i = 0; i < N - 1; i++) {
    auto u0 = input_at_knots.col(i);
    auto u1 = input_at_knots.col(i + 1);
    auto h = time_at_knots(i + 1) - time_at_knots(i);
    cost_u += ((u0.transpose() * W_R * u0) * h / 2)(0);
    cost_u += ((u1.transpose() * W_R * u1) * h / 2)(0);
  }
  total_cost += cost_u;
  cout << "cost_u = " << cost_u << endl;
  double cost_lambda = 0;
  for (unsigned int i = 0; i < num_time_samples.size(); i++) {
    for (int j = 0; j < num_time_samples[i]; j++) {
      auto lambda = result.GetSolution(trajopt->force(i, j));
      cost_lambda += (options_list[i].getForceCost() * lambda).squaredNorm();
    }
  }
  total_cost += cost_lambda;
  cout << "cost_lambda = " << cost_lambda << endl;
  // cost on force difference wrt time
  double cost_lambda_diff = 0;
  for (int i = 0; i < N - 1; i++) {
    auto lambda0 = result.GetSolution(trajopt->force(0, i));
    auto lambda1 = result.GetSolution(trajopt->force(0, i + 1));
    auto lambdac = result.GetSolution(trajopt->collocation_force(0, i));
    if (diff_with_force_at_collocation) {
      cost_lambda_diff +=
          w_lambda_diff * (lambda0 - lambdac).dot(lambda0 - lambdac);
      cost_lambda_diff +=
          w_lambda_diff * (lambdac - lambda1).dot(lambdac - lambda1);
    } else {
      cost_lambda_diff +=
          w_lambda_diff * (lambda0 - lambda1).dot(lambda0 - lambda1);
    }
  }
  total_cost += cost_lambda_diff;
  cout << "cost_lambda_diff = " << cost_lambda_diff << endl;
  // cost on vel difference wrt time
  double cost_vel_diff = 0;
  for (int i = 0; i < N - 1; i++) {
    auto v0 = result.GetSolution(trajopt->state(i).tail(n_v));
    auto v1 = result.GetSolution(trajopt->state(i + 1).tail(n_v));
    cost_vel_diff += (v0 - v1).dot(Q_v_diff * (v0 - v1));
  }
  total_cost += cost_vel_diff;
  cout << "cost_vel_diff = " << cost_vel_diff << endl;
  // cost on input difference wrt time
  double cost_u_diff = 0;
  for (int i = 0; i < N - 1; i++) {
    auto u0 = result.GetSolution(trajopt->input(i));
    auto u1 = result.GetSolution(trajopt->input(i + 1));
    cost_u_diff += w_u_diff * (u0 - u1).dot(u0 - u1);
  }
  total_cost += cost_u_diff;
  cout << "cost_u_diff = " << cost_u_diff << endl;
  // add cost on joint position
  double cost_q_hip_roll = 0;
  for (int i = 0; i < N; i++) {
    auto q1 =
        result.GetSolution(trajopt->state(i).segment<1>(hip_roll_left_idx));
    auto q2 =
        result.GetSolution(trajopt->state(i).segment<1>(hip_roll_right_idx));
    cost_q_hip_roll += w_q_hip_roll * q1.transpose() * q1;
    cost_q_hip_roll += w_q_hip_roll * q2.transpose() * q2;
  }
  total_cost += cost_q_hip_roll;
  cout << "cost_q_hip_roll = " << cost_q_hip_roll << endl;
  double cost_q_hip_yaw = 0;
  for (int i = 0; i < N; i++) {
    auto q1 =
        result.GetSolution(trajopt->state(i).segment<1>(hip_yaw_left_idx));
    auto q2 =
        result.GetSolution(trajopt->state(i).segment<1>(hip_yaw_right_idx));
    cost_q_hip_roll += w_q_hip_yaw * q1.transpose() * q1;
    cost_q_hip_roll += w_q_hip_yaw * q2.transpose() * q2;
  }
  total_cost += cost_q_hip_yaw;
  cout << "cost_q_hip_yaw = " << cost_q_hip_yaw << endl;
  // add cost on quaternion
  double cost_q_quat_xyz = 0;
  for (int i = 0; i < N; i++) {
    auto q = result.GetSolution(trajopt->state(i).segment(base_qx_idx, 3));
    cost_q_quat_xyz += w_q_quat_xyz * q.transpose() * q;
  }
  total_cost += cost_q_quat_xyz;
  cout << "cost_q_quat_xyz = " << cost_q_quat_xyz << endl;

  cout << "total_cost = " << total_cost << endl;

  // Save trajectory to file
  DirconTrajectory saved_traj(plant, *trajopt, result, "walking_trajectory",
                              "Decision variables and state/input trajectories "
                              "for walking");
  saved_traj.WriteToFile(FLAGS_data_directory + FLAGS_save_filename);
  std::cout << "Wrote to file: " << FLAGS_data_directory + FLAGS_save_filename
            << std::endl;

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
  auto ball_plant = multibody::ConstructBallPlant(&scene_graph);
  if (plot_com) {
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

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
}
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain(FLAGS_duration, FLAGS_stride_length, FLAGS_ground_incline,
                  FLAGS_is_fix_time, FLAGS_n_node, FLAGS_max_iter,
                  FLAGS_data_directory, FLAGS_init_file, FLAGS_tol,
                  FLAGS_store_data, FLAGS_scale_option);
}
