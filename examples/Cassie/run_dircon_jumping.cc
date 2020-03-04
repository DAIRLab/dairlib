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
using std::unordered_map;
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
DEFINE_double(tol, 1e-6, "Tolerance for constraint violation and dual gap");
DEFINE_double(jump_height, 0.0, "Height to jump (m)");
DEFINE_string(load_filename, "", "File to load decision vars from.");
DEFINE_string(data_directory, "/home/yangwill/Documents/research/",
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

MatrixXd generate_state_input_matrix(const PiecewisePolynomial<double>& states,
                                     const PiecewisePolynomial<double>& inputs,
                                     VectorXd times);
vector<string> createStateNameVectorFromMap(const map<string, int>& pos_map,
                                            const map<string, int>& vel_map,
                                            const map<string, int>& act_map);

// Position constraint of a body origin in one dimension (x, y, or z)
class OneDimBodyPosConstraint : public DirconAbstractConstraint<double> {
 public:
  OneDimBodyPosConstraint(const MultibodyPlant<double>* plant, string body_name,
                          int xyz_idx, double lb, double ub)
      : DirconAbstractConstraint<double>(
            1, plant->num_positions(), VectorXd::Ones(1) * lb,
            VectorXd::Ones(1) * ub, body_name + "_constraint"),
        plant_(plant),
        body_(plant->GetBodyByName(body_name)),
        xyz_idx_(xyz_idx) {}
  ~OneDimBodyPosConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override {
    VectorXd q = x;

    std::unique_ptr<drake::systems::Context<double>> context =
        plant_->CreateDefaultContext();
    plant_->SetPositions(context.get(), q);

    VectorXd pt(3);
    this->plant_->CalcPointsPositions(*context, body_.body_frame(),
                                      Vector3d::Zero(), plant_->world_frame(),
                                      &pt);
    *y = pt.segment(xyz_idx_, 1);
  };

 private:
  const MultibodyPlant<double>* plant_;
  const drake::multibody::Body<double>& body_;
  // xyz_idx_ takes value of 0, 1 or 2.
  // 0 is x, 1 is y and 2 is z component of the position vector.
  const int xyz_idx_;
};

void DoMain() {
  // Drake system initialization stuff
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

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;

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
  auto double_stance_dataset = DirconKinematicDataSet<double>(
      plant, &double_stance_constraints, skip_constraint_inds);
  auto double_stance_options =
      DirconOptions(double_stance_dataset.countConstraints(), &plant);
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

  //   Flight mode (no contact)
  std::vector<DirconKinematicData<double>*> flight_mode_constraints;
  flight_mode_constraints.push_back(&distance_constraint_left);
  flight_mode_constraints.push_back(&distance_constraint_right);
  auto flight_mode_dataset =
      DirconKinematicDataSet<double>(plant, &flight_mode_constraints);
  auto flight_mode_options =
      DirconOptions(flight_mode_dataset.countConstraints(), &plant);

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
  contact_mode_list.push_back(&double_stance_dataset);

  options_list.push_back(double_stance_options);
  options_list.push_back(flight_mode_options);
  options_list.push_back(double_stance_options);

  double w_lambda = 1.0e-4;
  // set force cost weight
  for (int i = 0; i < 2; i++) {
    options_list[i].setForceCost(w_lambda);
  }

  vector<int> stance_modes{0, 2};

  //  for (int i : stance_modes) {
  //    double s = 1;  // scale everything together
  //    // Dynamic constraints
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 30.0, 0, 3);
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 60.0, 4, 5);
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 1500.0, 6, 16);
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 300.0, 17, 18);
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 600.0, 19, 28);
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 2000.0, 29, 34);
  //    options_list[i].setDynConstraintScaling(s * 1.0 / 2000.0, 35, 36);
  //    // Kinematic constraints
  //    options_list[i].setKinConstraintScaling(s * 1.0 / 50.0, 0, 4);
  //    options_list[i].setKinConstraintScaling(s * 1.0 / 600.0 * 2, 5, 6);
  //    options_list[i].setKinConstraintScaling(s * 1.0 / 1000.0, 7 + 0, 7 + 4);
  //    options_list[i].setKinConstraintScaling(s * 1.0, 7 + 5, 7 + 6);
  //    options_list[i].setKinConstraintScaling(s * 1.0, 14 + 0, 14 + 4);
  //    options_list[i].setKinConstraintScaling(s * 1.0 / 200, 14 + 5, 14 + 6);
  //    // Impact constraints
  //    options_list[i].setImpConstraintScaling(s * 1.0 / 50.0, 0, 2);
  //    options_list[i].setImpConstraintScaling(s * 1.0 / 300.0, 3, 5);
  //    options_list[i].setImpConstraintScaling(s * 1.0 / 24.0, 6, 7);
  //    options_list[i].setImpConstraintScaling(s * 1.0 / 6.0, 8, 9);
  //    options_list[i].setImpConstraintScaling(s * 1.0 / 12.0, 10, 13);
  //    options_list[i].setImpConstraintScaling(s * 1.0 / 2.0, 14, 15);
  //    options_list[i].setImpConstraintScaling(s * 1.0, 16, 17);
  //  }

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, timesteps, min_dt, max_dt, contact_mode_list, options_list);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../jumping_snopt.out");
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
  //  HybridDircon<double>* dircon = trajopt.get();
  std::cout << "Adding kinematic constraints: " << std::endl;
  setKinematicConstraints(trajopt.get(), plant);

  //  int num_knot_points = FLAGS_knot_points * n_modes - (n_modes - 1);
  std::cout << "Setting initial conditions: " << std::endl;
  vector<int> mode_lengths = trajopt->mode_lengths();

  int num_knot_points = trajopt->N();
  std::cout << "nq: " << n_q << endl;
  std::cout << "nv: " << n_v << endl;
  std::cout << "nu: " << plant.num_actuators() << endl;
  cout << "N: " << num_knot_points;
  cout << "Num decision vars: " << trajopt->decision_variables().size() << endl;
  double dt = 0.1;

  if (!FLAGS_load_filename.empty()) {
    std::cout << "Loading: " << FLAGS_load_filename << std::endl;
    MatrixXd decisionVars =
        loadSavedDecisionVars(FLAGS_data_directory + FLAGS_load_filename);
    trajopt->SetInitialGuessForAllVariables(decisionVars);
  } else {
    //    double timestep = rabbit_traj.end_time() / dircon->N();
    // Initialize all decision vars to random by default. Will be overriding
    // later
    trajopt->SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt->decision_variables().size()));
    // Set the initial guess for states

    vector<VectorXd> q_guess_stance =
        GetInitGuessForQStance(num_knot_points, plant);
    vector<VectorXd> q_guess_flight =
        GetInitGuessForQFlight(num_knot_points, FLAGS_jump_height, plant);
    vector<VectorXd> v_guess_stance =
        GetInitGuessForV(q_guess_stance, dt, plant);
    vector<VectorXd> v_guess_flight =
        GetInitGuessForV(q_guess_flight, dt, plant);
    // Set initial guess for stance
    VectorXd x_guess(n_q + n_v);
    for (int i = 0; i < mode_lengths[0]; ++i) {
      //      int knot_point = i;
      x_guess << q_guess_stance[i], v_guess_stance[i];
      trajopt->SetInitialGuess(trajopt->state(i), x_guess);
    }
    // Set initial guess for flight
    for (int i = 0; i < mode_lengths[1]; ++i) {
      int knot_point = mode_lengths[0] - 1 + i;
      x_guess << q_guess_flight[i], v_guess_flight[i];
      trajopt->SetInitialGuess(trajopt->state(knot_point), x_guess);
    }
    // Set initial guess for flight
    for (int i = 0; i < mode_lengths[2]; ++i) {
      int knot_point = mode_lengths[0] + mode_lengths[1] - 2 + i;
      x_guess << q_guess_stance[i], v_guess_stance[i];
      trajopt->SetInitialGuess(trajopt->state(knot_point), x_guess);
    }

    // Some guesses
    //    for (int i = 0; i < trajopt->N(); ++i) {
    //      auto u = trajopt->input(i);
    //      trajopt->SetInitialGuess(u(0), 30);
    //      trajopt->SetInitialGuess(u(2), -30);
    //      trajopt->SetInitialGuess(u(4), 30);
    //      trajopt->SetInitialGuess(u(6), 30);
    //      trajopt->SetInitialGuess(u(8), 30);
    //    }

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

  // Printing
  for (int i = 0; i < trajopt->decision_variables().size(); i++) {
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
    cout << trajopt->GetInitialGuess(trajopt->decision_variable(i));
    cout << endl;
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

  //  std::cout << result.GetSolution();

  const PiecewisePolynomial<double>& state_traj =
      trajopt->ReconstructStateTrajectory(result);
  const PiecewisePolynomial<double>& input_traj =
      trajopt->ReconstructInputTrajectory(result);

  LcmTrajectory::Trajectory decision_vars;
  decision_vars.traj_name = "cassie_jumping_decision_vars";
  decision_vars.datapoints = result.GetSolution();
  decision_vars.time_vector = VectorXd::Zero(decision_vars.datapoints.size());
  decision_vars.datatypes = vector<string>(decision_vars.datapoints.size());

  int num_modes = 3;
  std::vector<LcmTrajectory::Trajectory> trajectories;
  std::vector<std::string> trajectory_names;

  VectorXd segment_times(2 * num_modes);
  segment_times << 0, state_traj.get_segment_times().at(FLAGS_knot_points - 1),
      state_traj.get_segment_times().at(FLAGS_knot_points),
      state_traj.get_segment_times().at(2 * FLAGS_knot_points - 1),
      state_traj.get_segment_times().at(2 * FLAGS_knot_points),
      state_traj.end_time();
  for (int mode = 0; mode < num_modes; ++mode) {
    LcmTrajectory::Trajectory traj_block;
    traj_block.traj_name =
        "cassie_jumping_trajectory_x_u" + std::to_string(mode);
    //      traj_block.time_vector = generate_time_matrix(state_traj, 1000);
    traj_block.time_vector = VectorXd::LinSpaced(1000, segment_times(2 * mode),
                                                 segment_times(2 * mode + 1));
    traj_block.datapoints = generate_state_input_matrix(state_traj, input_traj,
                                                        traj_block.time_vector);
    traj_block.datatypes =
        createStateNameVectorFromMap(pos_map, vel_map, act_map);
    trajectories.push_back(traj_block);
    trajectory_names.push_back(traj_block.traj_name);
  }

  trajectories.push_back(decision_vars);
  trajectory_names.push_back(decision_vars.traj_name);
  LcmTrajectory saved_traj(trajectories, trajectory_names, "jumping_trajectory",
                           "Decision variables and state/input trajectories "
                           "for jumping");
  saved_traj.writeToFile(FLAGS_data_directory + FLAGS_save_filename);

  drake::trajectories::PiecewisePolynomial<double> optimal_traj =
      trajopt->ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                         optimal_traj);

  auto diagram = builder.Build();

  while (true) {
    //    std::this_thread::sleep_for(std::chrono::seconds(2));
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

  for (auto pair : pos_map) {
    std::cout << pair.first << ": " << pair.second << std::endl;
  }

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  int n_x = n_q + n_v;

  //  std::cout << "N: " << trajopt->N() << std::endl;
  // Get the decision variables that will be used
  int N = trajopt->N();
  auto mode_lengths = trajopt->mode_lengths();
  auto x0 = trajopt->initial_state();
  //  auto x_bottom = trajopt->state(FLAGS_knot_points / 2);
  auto x_top = trajopt->state(3 * FLAGS_knot_points / 2);
  //  auto x_mid_point =
  //      trajopt->state(FLAGS_knot_points * mode_lengths.size() / 2);
  auto xf = trajopt->final_state();
  auto u = trajopt->input();
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  //  auto lambda_init = trajopt->force_vars(0);
  //  auto lambda_final = trajopt->force_vars(2);
  auto x = trajopt->state();

  // Duration Bounds
  double min_duration = 1.0;
  double max_duration = 1.5;
  trajopt->AddDurationBounds(min_duration, max_duration);

  // Periodicity constraints

  // Jumping height constraints

  // Standing constraints
  double rest_height = 1.075;

  // Fixing initial conditions
  //  VectorXd q_init(n_q);
  //  q_init << 1, -0.000439869, -3.78298e-06, 0.00024498, 0.000994219, -0.001,
  //      1.20078, -0.00806937, 0.0140112, 0, 0, 0.583512, 0.532804, -1.17628,
  //      -1.18182, 1.40317, 1.40871, -1.3, -1.3;
  //  trajopt->AddBoundingBoxConstraint(q_init, q_init, x0.head(n_q));
  //  trajopt->AddBoundingBoxConstraint(q_init, q_init, xf.head(n_q));

  // Just position constraints
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  //  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_y")));
  //  trajopt->AddBoundingBoxConstraint(0, 0, xf(pos_map.at("base_y")));
  //  trajopt->AddBoundingBoxConstraint(0, 0, x_top(pos_map.at("base_x")));
  //  trajopt->AddBoundingBoxConstraint(0, 0, x_top(pos_map.at("base_y")));

  // Knee Angles
  //  trajopt->AddBoundingBoxConstraint(-0.85, -0.75,
  //  x0(pos_map.at("knee_left"))); trajopt->AddBoundingBoxConstraint(-0.85,
  //  -0.75, x0(pos_map.at("knee_right")));
  // Hip orientations
  //  trajopt->AddBoundingBoxConstraint(-1e-3, 1e-3,
  //                                    x0(pos_map.at("hip_roll_left")));
  //  trajopt->AddBoundingBoxConstraint(-1e-3, 1e-3,
  //                                    x0(pos_map.at("hip_roll_right")));
  //  trajopt->AddBoundingBoxConstraint(-1e-3, 1e-3,
  //                                    xf(pos_map.at("hip_roll_left")));
  //  trajopt->AddBoundingBoxConstraint(-1e-3, 1e-3,
  //                                    xf(pos_map.at("hip_roll_right")));
  //  trajopt->AddBoundingBoxConstraint(-1e-2, 1e-2,
  //                                    x0(pos_map.at("hip_yaw_left")));
  //  trajopt->AddBoundingBoxConstraint(-1e-2, 1e-2,
  //                                    x0(pos_map.at("hip_yaw_right")));
  //  trajopt->AddBoundingBoxConstraint(-1e-2, 1e-2,
  //                                    xf(pos_map.at("hip_yaw_left")));
  //  trajopt->AddBoundingBoxConstraint(-1e-2, 1e-2,
  //                                    xf(pos_map.at("hip_yaw_right")));

  // Jumping height constraints
  trajopt->AddBoundingBoxConstraint(rest_height, rest_height,
                                    x0(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(FLAGS_jump_height + rest_height,
                                    FLAGS_jump_height + rest_height,
                                    x_top(pos_map.at("base_z")));
  trajopt->AddBoundingBoxConstraint(rest_height, rest_height,
                                    xf(pos_map.at("base_z")));

  // Zero starting and final velocities
  trajopt->AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                    x0.tail(n_v));
  trajopt->AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                    xf.tail(n_v));

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

  std::cout << "Joint symmetric constraints: " << std::endl;
  for (const auto& l_r_pair : l_r_pairs) {
    for (const auto& sym_joint_name : sym_joint_names) {
      trajopt->AddLinearConstraint(
          x0(pos_map[sym_joint_name + l_r_pair.first]) ==
          x0(pos_map[sym_joint_name + l_r_pair.second]));
      trajopt->AddLinearConstraint(
          xf(pos_map[sym_joint_name + l_r_pair.first]) ==
          xf(pos_map[sym_joint_name + l_r_pair.second]));
      if (sym_joint_name != "ankle_joint") {  // No actuator at ankle
        trajopt->AddLinearConstraint(
            u0(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            u0(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
        trajopt->AddLinearConstraint(
            uf(act_map.at(sym_joint_name + l_r_pair.first + "_motor")) ==
            uf(act_map.at(sym_joint_name + l_r_pair.second + "_motor")));
      }
    }

    for (const auto& asy_joint_name : asy_joint_names) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(asy_joint_name + l_r_pair.first)) ==
          xf(pos_map.at(asy_joint_name + l_r_pair.second)));

      // inputs
      trajopt->AddLinearConstraint(
          u0(act_map.at(asy_joint_name + l_r_pair.first + "_motor")) ==
          uf(act_map.at(asy_joint_name + l_r_pair.second + "_motor")));
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

  // actuator limit
  std::cout << "Actuator limit constraints: " << std::endl;
  for (int i = 0; i < trajopt->N(); i++) {
    auto ui = trajopt->input(i);
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  // toe position constraint in y direction (avoid leg crossing)
  // tighter constraint than before
  auto left_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_left", 1, 0.05, 0.3);
  //      &plant, "toe_left", 1, 0.05, std::numeric_limits<double>::infinity());
  auto right_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_right", 1, -0.3, -0.05);
  //      &plant, "toe_right", 1, -std::numeric_limits<double>::infinity(),
  //      -0.05);
  for (int index = 0; index < mode_lengths[0]; index++) {
    auto x = trajopt->state(index);
    trajopt->AddConstraint(left_foot_constraint, x.head(n_q));
    trajopt->AddConstraint(right_foot_constraint, x.head(n_q));
  }

  // Only add constraints of lambdas for stance modes
  vector<int> stance_modes{0, 2};
  //  for (int mode : stance_modes) {
  for (int index = 0; index < mode_lengths[0]; index++) {
    auto lambda = trajopt->force(0, index);
    trajopt->AddLinearConstraint(lambda(2) >= 10);
    trajopt->AddLinearConstraint(lambda(5) >= 10);
    trajopt->AddLinearConstraint(lambda(8) >= 10);
    trajopt->AddLinearConstraint(lambda(11) >= 10);
  }
  for (int index = 0; index < mode_lengths[2]; index++) {
    auto lambda = trajopt->force(2, index);
    trajopt->AddLinearConstraint(lambda(2) <= 70);
    trajopt->AddLinearConstraint(lambda(5) <= 70);
    trajopt->AddLinearConstraint(lambda(8) <= 70);
    trajopt->AddLinearConstraint(lambda(11) <= 70);
  }
  //  }

  const MatrixXd Q = 0.1 * MatrixXd::Identity(n_v, n_v);
  const MatrixXd R = 0.005 * MatrixXd::Identity(n_u, n_u);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  //  bool constrain_stance_leg_fourbar_force = true;
  //  // Testing -- constraint left four-bar force (seems to help in high speed)
  //  if (constrain_stance_leg_fourbar_force) {
  //    for (int mode : stance_modes) {
  //      for (int index = 0; index < mode_lengths[mode]; index++) {
  //        auto lambda = trajopt->force(mode, index);
  //        trajopt->AddLinearConstraint(lambda(12) <= 0);  // left leg four bar
  //        trajopt->AddLinearConstraint(lambda(13) <= 0);  // left leg four bar
  //      }
  //    }
  //    // Different indices for flight mode
  //    for (int index = 0; index < mode_lengths[1]; index++) {
  //      auto lambda = trajopt->force(1, index);
  //      trajopt->AddLinearConstraint(lambda(0) <= 0);  // left leg four bar
  //      trajopt->AddLinearConstraint(lambda(1) <= 0);  // left leg four bar
  //    }
  //  }

  // Add some cost to hip roll and yaw
  double w_q_hip_roll = 0.3;
  double w_q_hip_yaw = 0.3;
  double w_q_hip_pitch = 0.5;
  double w_q_quat_xyz = 0.3;
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
      trajopt->AddCost(w_q_hip_yaw * q.transpose() * q);
    }
  }
  if (w_q_quat_xyz) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(1, 3);
      trajopt->AddCost(w_q_quat_xyz * q.transpose() * q);
    }
  }

  // Scale decision variable
  //  double s_q_toe = 1;
  //  double s_v_toe_l = 1;
  //  double s_v_toe_r = 1;
  // time
  //  trajopt->ScaleTimeVariables(0.008);
  // state
  //  trajopt->ScaleStateVariables(0.5, 0, 3);
  //  if (s_q_toe > 1) {
  //    trajopt->ScaleStateVariables(s_q_toe, n_q - 2, n_q - 1);
  //  }
  //  trajopt->ScaleStateVariables(10, n_q, n_q + n_v - 3);
  //  trajopt->ScaleStateVariables(10 * s_v_toe_l, n_q + n_v - 2, n_q + n_v -
  //  2); trajopt->ScaleStateVariables(10 * s_v_toe_r, n_q + n_v - 1, n_q + n_v
  //  - 1);
  // input
  //  trajopt->ScaleInputVariables(100, 0, 9);
  // force
  //  trajopt->ScaleForceVariables(1000, 0, 0,
  //                               trajopt->num_kinematic_constraints(0) - 1);
  //  trajopt->ScaleForceVariables(1000, 2, 0,
  //                               trajopt->num_kinematic_constraints(2) - 1);
  // impulse
  //  trajopt->ScaleImpulseVariables(
  //      10, 0, 0, trajopt->num_kinematic_constraints(0) - 1);  // 0.1
  //  trajopt->ScaleImpulseVariables(
  //      10, 2, 0, trajopt->num_kinematic_constraints(2) - 1);  // 0.1
  // quaternion slack
  //  trajopt->ScaleQuaternionSlackVariables(150);
  // Constraint slack
  //  trajopt->ScaleKinConstraintSlackVariables(50, 0, 0, 5);
  //  trajopt->ScaleKinConstraintSlackVariables(500, 0, 6, 7);
}

vector<VectorXd> GetInitGuessForQStance(int num_knot_points,
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
    Vector3d pelvis_pos(
        0.0, 0.0,
        1.0 + 0.01 * (i - num_knot_points / 2) * (i - num_knot_points / 2));
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
  double rest_height = 1.125;

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
  bool standing = false;
  if (!standing) {
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
  } else {
    vector<VectorXd> v_seed;
    for (unsigned int i = 0; i < q_guess.size(); ++i) {
      VectorXd zeros = VectorXd::Zero(plant.num_velocities());

      v_seed.push_back(zeros);
    }
    return v_seed;
  }
}

MatrixXd loadSavedDecisionVars(const string& filepath) {
  const LcmTrajectory& loaded_decision_vars = LcmTrajectory(filepath);
  return loaded_decision_vars.getTrajectory("cassie_jumping_decision_vars")
      .datapoints;
}

MatrixXd generate_state_input_matrix(const PiecewisePolynomial<double>& states,
                                     const PiecewisePolynomial<double>& inputs,
                                     VectorXd times) {
  int num_states = states.value(0).size();
  int num_inputs = inputs.value(0).size();
  MatrixXd states_matrix = MatrixXd::Ones(num_states, times.size());
  MatrixXd inputs_matrix = MatrixXd::Ones(num_inputs, times.size());

  for (int i = 0; i < times.size(); ++i) {
    states_matrix.col(i) = states.value(times[i]);
    inputs_matrix.col(i) = inputs.value(times[i]);
  }
  MatrixXd states_and_inputs(num_states + num_inputs, times.size());
  states_and_inputs.topRows(num_states) = states_matrix;
  states_and_inputs.bottomRows(num_inputs) = inputs_matrix;

  return states_and_inputs;
}

vector<string> createStateNameVectorFromMap(const map<string, int>& pos_map,
                                            const map<string, int>& vel_map,
                                            const map<string, int>& act_map) {
  vector<string> state_names(pos_map.size() + vel_map.size() + act_map.size());

  for (const auto& name_index_pair : pos_map) {
    state_names[name_index_pair.second] = name_index_pair.first;
  }
  for (const auto& name_index_pair : vel_map) {
    state_names[name_index_pair.second + pos_map.size()] =
        name_index_pair.first;
  }
  for (const auto& name_index_pair : act_map) {
    state_names[name_index_pair.second + pos_map.size() + vel_map.size()] =
        name_index_pair.first;
  }
  return state_names;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}