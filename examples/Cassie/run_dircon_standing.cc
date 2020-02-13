#include <chrono>
#include <memory>
#include <string>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "drake/multibody/parsing/parser.h"

#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/choose_best_solver.h>
#include <drake/solvers/snopt_solver.h>
#include <drake/systems/analysis/simulator.h>
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "drake/solvers/solve.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

using std::cout;
using std::endl;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
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
DEFINE_double(height, 0.3, "Target height for jumping.");
DEFINE_double(land_height, 0.0, "End height relative to initial height.");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");
DEFINE_string(filename, "", "File to load decision vars from.");
DEFINE_string(traj_name, "", "File to load saved LCM trajs from.");

namespace dairlib {

shared_ptr<HybridDircon<double>> createDircon(MultibodyPlant<double>& plant);

void setKinematicConstraints(HybridDircon<double>* trajopt,
                             MultibodyPlant<double>& plant);

MatrixXd generateInitialGuess(const Vector3d& CoM_pos, VectorXd* states,
                              VectorXd* inputs, VectorXd* lambdas,
                              MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForQ(int num_knot_points, double stride_length,
                                  const MultibodyPlant<double>& plant);
vector<VectorXd> GetInitGuessForV(const vector<VectorXd>& q_guess, double dt,
                                  const MultibodyPlant<double>& plant);
MatrixXd loadSavedDecisionVars(string filename);

void DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(1e-4);
  Parser parser(&plant, &scene_graph);

  string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.Finalize();

  std::cout << "Creating dircon object: " << std::endl;
  auto trajopt = createDircon(plant);
  HybridDircon<double>* dircon = trajopt.get();
  std::cout << "Adding kinematic constraints: " << std::endl;
  setKinematicConstraints(dircon, plant);

  //  int num_knot_points = FLAGS_knot_points * n_modes - (n_modes - 1);
  std::cout << "Setting initial conditions: " << std::endl;
  int n_modes = 1;
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int num_knot_points = dircon->N();
  double stride_length = 0.3;
  double dt = 0.1;
  vector<VectorXd> q_guess =
      GetInitGuessForQ(num_knot_points, stride_length, plant);
  vector<VectorXd> v_guess = GetInitGuessForV(q_guess, dt, plant);

  //  const LcmTrajectory& loaded_traj =
  //  LcmTrajectory(LcmTrajectory::loadFromFile(
  //      "examples/jumping/saved_trajs/" + FLAGS_traj_name));
  //
  //  const LcmTrajectory::Trajectory& com_traj =
  //      loaded_traj.getTrajectory("center_of_mass_trajectory");
  //  const PiecewisePolynomial<double>& rabbit_traj =
  //      PiecewisePolynomial<double>::Cubic(com_traj.time_vector,
  //          com_traj.datapoints);

  if (!FLAGS_filename.empty()) {
    std::cout << "Loading: " << FLAGS_filename << std::endl;
    string folder_path = "saved_trajs/";
    MatrixXd decisionVars = loadSavedDecisionVars(folder_path + FLAGS_filename);
    dircon->SetInitialGuessForAllVariables(decisionVars);
  } else {
    //    double timestep = rabbit_traj.end_time() / dircon->N();
    // Initialize all decision vars to random by default. Will be overriding
    // later
    trajopt->SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt->decision_variables().size()));
    DRAKE_ASSERT(q_guess.size() == dircon->N());
    DRAKE_ASSERT(v_guess.size() == dircon->N());
    // Set the initial guess for states
    for (int i = 0; i < dircon->N(); ++i) {
      VectorXd x_guess(n_q + n_v);
      x_guess << q_guess[i], v_guess[i];
      dircon->SetInitialGuess(dircon->state(i), x_guess);
    }

    // Some guesses
    for (int i = 0; i < dircon->N(); ++i) {
      auto u = dircon->input(i);
      dircon->SetInitialGuess(u(0), 30);
      dircon->SetInitialGuess(u(2), -30);
      dircon->SetInitialGuess(u(4), 30);
      dircon->SetInitialGuess(u(6), 30);
      dircon->SetInitialGuess(u(8), 30);
    }

    //    for (int mode = 0; mode < n_modes; ++mode) {
    //      for (int i = 0; i < FLAGS_knot_points; ++i) {
    //        auto lambdas = dircon->force(mode, i);
    //
    //        dircon->SetInitialGuess(lambdas(0), 15);
    //        dircon->SetInitialGuess(lambdas(1), 15);
    //        dircon->SetInitialGuess(lambdas(2), 15);
    //        dircon->SetInitialGuess(lambdas(3), 15);
    //        dircon->SetInitialGuess(lambdas(4), 15);
    //      }
    //    }
  }

  // To avoid NaN quaternions
  for (int i = 0; i < dircon->N(); i++) {
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
  //  SolutionResult solution_result = result.get_solution_result();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << '\a';  // Beep
  cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;
  std::cout << "Solve result: " << result.get_solution_result() << std::endl;
}

MatrixXd generateInitialGuess(const Vector3d& CoM_pos, VectorXd* states,
                              VectorXd* inputs, VectorXd* lambdas,
                              MultibodyPlant<double>& plant) {
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  int n_x = n_q + n_v;

  *states = VectorXd::Random(n_x);
  *inputs = VectorXd::Random(n_u);
  *lambdas = VectorXd::Random(n_x);

  return MatrixXd::Identity(3, 3);
}

vector<VectorXd> GetInitGuessForQ(int num_knot_points, double stride_length,
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

  for (int i = 0; i < num_knot_points; i++) {
    double eps = 1e-3;
    Vector3d eps_vec = eps * VectorXd::Ones(3);
    //    Vector3d pelvis_pos(stride_length * i / (num_knot_points - 1),
    //    0.0, 1.0);
    Vector3d pelvis_pos(0.0, 0.0, 1.0);
    Vector3d left_toe_pos(0.0, 0.12, 0.05);
    Vector3d right_toe_pos(0.0, -0.12, 0.05);

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

    bool visualize_init_traj = true;
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
                                  const MultibodyPlant<double>& plant) {
  vector<VectorXd> qdot_guess;
  qdot_guess.push_back((q_guess[1] - q_guess[0]) / dt);
  for (unsigned int i = 1; i < q_guess.size() - 1; i++) {
    VectorXd v_plus = (q_guess[i + 1] - q_guess[i]) / dt;
    VectorXd v_minus = (q_guess[i] - q_guess[i - 1]) / dt;
    qdot_guess.push_back((v_plus + v_minus) / 2);
  }
  qdot_guess.push_back((q_guess.back() - q_guess[q_guess.size() - 2]) / dt);

  // Convert qdot to v
  DRAKE_ASSERT(qdot_guess.size() == q_guess.size());
  vector<VectorXd> v_seed;
  for (unsigned int i = 0; i < q_guess.size(); i++) {
    auto context = plant.CreateDefaultContext();
    plant.SetPositions(context.get(), q_guess[i]);
    VectorXd v(plant.num_velocities());
    plant.MapQDotToVelocity(*context, qdot_guess[i], &v);
    v_seed.push_back(v);
  }
  return v_seed;
}

MatrixXd loadSavedDecisionVars(string filename) { return MatrixXd::Zero(3, 3); }

shared_ptr<HybridDircon<double>> createDircon(MultibodyPlant<double>& plant) {
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

  // Double stance (all four contact points)
  vector<DirconKinematicData<double>*> double_stance_constraints;
  double_stance_constraints.push_back(&left_toe_front_constraint);
  double_stance_constraints.push_back(&left_toe_rear_constraint);
  double_stance_constraints.push_back(&right_toe_front_constraint);
  double_stance_constraints.push_back(&right_toe_rear_constraint);
  double_stance_constraints.push_back(&distance_constraint_left);
  double_stance_constraints.push_back(&distance_constraint_right);
  auto double_stance_dataset = DirconKinematicDataSet<double>(
      plant, &double_stance_constraints, skip_constraint_inds);
  auto double_stance_options =
      DirconOptions(double_stance_dataset.countConstraints());
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

  // Flight mode (no contact)
  std::vector<DirconKinematicData<double>*> flight_mode_constraints;
  auto flight_mode_dataset =
      DirconKinematicDataSet<double>(plant, &flight_mode_constraints);
  auto flight_mode_options =
      DirconOptions(flight_mode_dataset.countConstraints());

  // timesteps and modes setting
  vector<int> timesteps;  // Number of timesteps per mode
  timesteps.push_back(FLAGS_knot_points);
  timesteps.push_back(FLAGS_knot_points);
  timesteps.push_back(FLAGS_knot_points);
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
  contact_mode_list.push_back(&double_stance_dataset);
  contact_mode_list.push_back(&flight_mode_dataset);
  contact_mode_list.push_back(&double_stance_dataset);

  std::vector<DirconOptions> options_list;
  options_list.push_back(double_stance_options);
  options_list.push_back(flight_mode_options);
  options_list.push_back(double_stance_options);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, timesteps, min_dt, max_dt, contact_mode_list, options_list);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../standing_snopt.out");
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
  return trajopt;
}

void setKinematicConstraints(HybridDircon<double>* trajopt,
                             MultibodyPlant<double>& plant) {
  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  int n_x = n_q + n_v;
  // Get the decision variables that will be used
  int n_modes = 3;
  int N = trajopt->N();
  auto x0 = trajopt->initial_state();
  auto x_mid_point = trajopt->state(FLAGS_knot_points * n_modes / 2);
  auto xf = trajopt->final_state();
  auto u = trajopt->input();
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto mode_lengths = trajopt->mode_lengths();
  auto lambda_init = trajopt->force_vars(0);
  auto lambda_final = trajopt->force_vars(2);
  auto x = trajopt->state();

  // Duration Bounds
  double min_duration = 1.0;
  double max_duration = 2.0;
  trajopt->AddDurationBounds(min_duration, max_duration);

  // Periodicity constraints

  //

  // Jumping height constraints

  // Standing constraints
  std::cout << "Position constraints: " << std::endl;
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(0.9, 1.0, xf(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(1.0, 1.0, x0(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(0.9, 0.9, xf(pos_map.at("base_z")));

  //  trajopt->AddLinearConstraint(x0 == initial_state);
  //  trajopt->AddLinearConstraint(x_mid_point(positions_map["planar_x"]) ==
  //      (x0(positions_map["planar_x"])));

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

  //  std::cout << "Joint symmetric constraints: " << std::endl;
  //  for (const auto& l_r_pair : l_r_pairs) {
  //    for (const auto& sym_joint_name : sym_joint_names) {
  //      trajopt->AddLinearConstraint(
  //          x0(pos_map[sym_joint_name + l_r_pair.first]) ==
  //          x0(pos_map[sym_joint_name + l_r_pair.second]));
  //      trajopt->AddLinearConstraint(
  //          xf(pos_map[sym_joint_name + l_r_pair.first]) ==
  //          xf(pos_map[sym_joint_name + l_r_pair.second]));
  //      trajopt->AddLinearConstraint(
  //          x0(n_q + pos_map[sym_joint_name + l_r_pair.first + "dot"]) ==
  //          x0(n_q + pos_map[sym_joint_name + l_r_pair.second + "dot"]));
  //      trajopt->AddLinearConstraint(
  //          xf(n_q + pos_map[sym_joint_name + l_r_pair.first + "dot"]) ==
  //          xf(n_q + pos_map[sym_joint_name + l_r_pair.second + "dot"]));
  //      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
  //        trajopt->AddLinearConstraint(
  //            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
  //            u0(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
  //        trajopt->AddLinearConstraint(
  //            uf(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
  //            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
  //      }
  //    }
  //  }

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

  // actuator limit
  std::cout << "Actuator limit constraints: " << std::endl;
  for (int i = 0; i < trajopt->N(); i++) {
    auto ui = trajopt->input(i);
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  //  for (unsigned int mode = 0; mode < n_modes; mode++) {
  //    for (int index = 0; index < mode_lengths[mode]; index++) {
  //      if (!(is_disable_kin_constraint_at_last_node &&
  //          (mode == mode_lengths.size() - 1))) {
  //        auto lambda = trajopt->force(mode, index);
  //        trajopt->AddLinearConstraint(lambda(2) >= 10);
  //        trajopt->AddLinearConstraint(lambda(5) >= 10);
  //      }
  //    }
  //  }

  const MatrixXd Q = 10 * 12.5 * MatrixXd::Identity(n_v, n_v);
  const MatrixXd R = 12.5 * MatrixXd::Identity(n_u, n_u);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}