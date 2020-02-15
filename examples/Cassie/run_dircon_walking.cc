#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <gflags/gflags.h>
#include "common/find_resource.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/com_pose_system.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/goldilocks_models/file_utils.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
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

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::goldilocks_models::readCSV;
using dairlib::goldilocks_models::writeCSV;
using dairlib::multibody::GetBodyIndexFromName;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;
using drake::VectorX;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

DEFINE_string(init_file, "", "the file name of initial guess");
DEFINE_string(data_directory, "../dairlib_data/cassie_trajopt_data/",
              "directory to save/read data");
DEFINE_bool(store_data, false, "To store solution or not");
DEFINE_int32(max_iter, 100000, "Iteration limit");
DEFINE_double(duration, 0.4, "Duration of the single support phase (s)");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");
DEFINE_int32(scale_option, 0,
             "Scale option of SNOPT"
             "Use 2 if seeing snopta exit 40 in log file");

// Parameters which enable dircon-improving features
DEFINE_bool(is_scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(is_scale_variable, true, "Scale the decision variable");

namespace dairlib {

/// Trajectory optimization of fixed-spring cassie walking
/// With the default initial guess, the solving time is about 2 mins.

// Do inverse kinematics to get configuration guess
vector<VectorXd> GetInitGuessForQ(int N, double stride_length,
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

  for (int i = 0; i < N; i++) {
    double eps = 1e-3;
    Vector3d eps_vec = eps * VectorXd::Ones(3);
    Vector3d pelvis_pos(stride_length * i / (N - 1), 0.0, 1.0);
    Vector3d stance_toe_pos(stride_length / 2, 0.12, 0.05);
    Vector3d swing_toe_pos(-stride_length / 2 + 2 * stride_length * i / (N - 1),
                           -0.12,
                           0.05 + 0.1 * (-abs((i - N / 2.0) / (N / 2.0)) + 1));
    // cout << "swing foot height = " <<
    //      0.05 + 0.1 * (-abs((i - N / 2.0) / (N / 2.0)) + 1);

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
    // SolutionResult solution_result = result.get_solution_result();
    // cout << "\n" << to_string(solution_result) << endl;
    // cout << "  Cost:" << result.get_optimal_cost() << std::endl;
    const auto q_sol = result.GetSolution(ik.q());
    // cout << "  q_sol = " << q_sol.transpose() << endl;
    // cout << "  q_sol.head(4).norm() = " << q_sol.head(4).norm() << endl;
    VectorXd q_sol_normd(n_q);
    q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
    // cout << "  q_sol_normd = " << q_sol_normd << endl;
    q_ik_guess = q_sol_normd;
    q_init_guess.push_back(q_sol_normd);

    bool visualize_init_traj = false;
    if (visualize_init_traj) {
      // Build temporary diagram for visualization
      drake::systems::DiagramBuilder<double> builder_ik;
      SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<SceneGraph>();
      scene_graph_ik.set_name("scene_graph_ik");
      MultibodyPlant<double> plant_ik(0.0);
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
    // cout << i << ":\n";
    // cout << "  qdot = " << qdot_seed[i].transpose() << endl;
    // cout << "  v = " << v.transpose() << endl;
  }
  return v_seed;
}

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

    VectorX<double> pt(3);
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

void DoMain(double duration, int max_iter, string data_directory,
            string init_file, double tol, bool to_store_data,
            int scale_option) {
  // parameters
  // double duration = 0.4;
  double stride_length = 0.2;
  double ground_incline = 0;

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
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();
  int n_x = n_q + n_v;

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

  // Disable kinematic constraint at the second node
  bool is_disable_kin_constraint_at_last_node = true;

  // Set up options
  std::vector<DirconOptions> options_list;
  options_list.push_back(DirconOptions(ls_dataset.countConstraints(), plant));
  options_list.push_back(DirconOptions(rs_dataset.countConstraints(), plant));

  if (is_disable_kin_constraint_at_last_node) {
    options_list[1].setSinglePeriodicEndNode(true);
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
  // Constraint scaling
  if (FLAGS_is_scale_constraint) {
    for (int i = 0; i < 2; i++) {
      double s = 1;  // scale everything together
      // Dynamic constraints
      options_list[i].setDynConstraintScaling(s * 1.0 / 30.0, 0, 3);
      options_list[i].setDynConstraintScaling(s * 1.0 / 60.0, 4, 16);
      options_list[i].setDynConstraintScaling(s * 1.0 / 300.0, 17, 18);
      options_list[i].setDynConstraintScaling(s * 1.0 / 600.0, 19, 28);
      options_list[i].setDynConstraintScaling(s * 1.0 / 3000.0, 29, 34);
      options_list[i].setDynConstraintScaling(s * 1.0 / 60000.0, 35, 36);
      // Kinematic constraints
      options_list[i].setKinConstraintScaling(s * 1.0 / 6000.0, 0, 4);
      options_list[i].setKinConstraintScaling(s * 1.0 / 600.0 * 2, 5, 6);
      options_list[i].setKinConstraintScaling(s * 1.0 / 10.0, 7 + 0, 7 + 4);
      options_list[i].setKinConstraintScaling(s * 1.0, 7 + 5, 7 + 6);
      options_list[i].setKinConstraintScaling(s * 1.0, 14 + 0, 14 + 4);
      options_list[i].setKinConstraintScaling(s * 1.0 * 20, 14 + 5, 14 + 6);
      // Impact constraints
      options_list[i].setImpConstraintScaling(s * 1.0 / 50.0, 0, 2);
      options_list[i].setImpConstraintScaling(s * 1.0 / 300.0, 3, 5);
      options_list[i].setImpConstraintScaling(s * 1.0 / 24.0, 6, 7);
      options_list[i].setImpConstraintScaling(s * 1.0 / 6.0, 8, 9);
      options_list[i].setImpConstraintScaling(s * 1.0 / 12.0, 10, 13);
      options_list[i].setImpConstraintScaling(s * 1.0 / 2.0, 14, 15);
      options_list[i].setImpConstraintScaling(s * 1.0, 16, n_v - 1);

      /* // old scaling from goldilocks model branch
      // Dynamic constraints
      options_list[i].setDynConstraintScaling(1.0 / 60.0, 0, n_q - 1);
      options_list[i].setDynConstraintScaling(1.0 / 1200.0, n_q, n_x - 1);
      // Kinematic constraints
      options_list[i].setKinConstraintScaling(1.0 / 600.0, 0, 4);
      options_list[i].setKinConstraintScaling(1.0 / 600.0 * 2, 5, 6);
      options_list[i].setKinConstraintScaling(1.0 / 10.0, 7 + 0, 7 + 4);
      options_list[i].setKinConstraintScaling(1.0 / 10.0, 7 + 5, 7 + 6);
      options_list[i].setKinConstraintScaling(1.0, 14 + 0, 14 + 4);
      options_list[i].setKinConstraintScaling(1.0 * 20, 14 + 5, 14 + 6);
      // Impact constraints
      options_list[i].setImpConstraintScaling(1.0 / 12.0 / 50.0, 0, 2);
      options_list[i].setImpConstraintScaling(1.0 / 12.0, 3, n_v - 1);*/
    }
  }

  // timesteps and modes setting
  vector<double> min_dt;
  vector<double> max_dt;
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  vector<int> num_time_samples;
  num_time_samples.push_back(16);  // 20
  num_time_samples.push_back(1);
  vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&ls_dataset);
  dataset_list.push_back(&rs_dataset);

  auto trajopt = std::make_shared<HybridDircon<double>>(
      plant, num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // Snopt settings
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "../snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", max_iter);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Iterations limit", 100000);  // QP subproblems
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0
  trajopt->SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Scale option",
      scale_option);  // snopt doc said try 2 if seeing snopta exit 40
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
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);

  // Fix time duration
  trajopt->AddDurationBounds(duration, duration);

  // x position constraint
  trajopt->AddBoundingBoxConstraint(0, 0, x0(pos_map.at("base_x")));
  trajopt->AddBoundingBoxConstraint(stride_length, stride_length,
                                    xf(pos_map.at("base_x")));

  // height constraint
  //  trajopt->AddLinearConstraint(x0(pos_map.at("base_z")) == 1);
  //  trajopt->AddLinearConstraint(xf(pos_map.at("base_z")) == 1.1);

  // initial pelvis position
  // trajopt->AddLinearConstraint(x0(pos_map.at("base_y")) == 0);

  // pelvis pose constraints
  //  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("base_qw")) ==
  //  1); trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("base_qx"))
  //  == 0);
  //  trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("base_qy")) ==
  //  0); trajopt->AddConstraintToAllKnotPoints(x(pos_map.at("base_qz"))
  //  == 0);

  // start/end velocity constraints
  //  trajopt->AddLinearConstraint(x0.tail(n_v) == VectorXd::Zero(n_v));
  //  trajopt->AddLinearConstraint(xf.tail(n_v) == VectorXd::Zero(n_v));

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
  if (ground_incline == 0) {
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
  for (auto l_r_pair : l_r_pairs) {
    for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(asy_joint_names[i] + l_r_pair.first)) ==
          -xf(pos_map.at(asy_joint_names[i] + l_r_pair.second)));
      // velocities
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(asy_joint_names[i] + l_r_pair.first + "dot")) ==
          -xf(n_q + vel_map.at(asy_joint_names[i] + l_r_pair.second + "dot")));
      // inputs
      trajopt->AddLinearConstraint(
          u0(act_map.at(asy_joint_names[i] + l_r_pair.first + "_motor")) ==
          -uf(act_map.at(asy_joint_names[i] + l_r_pair.second + "_motor")));
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      // positions
      trajopt->AddLinearConstraint(
          x0(pos_map.at(sym_joint_names[i] + l_r_pair.first)) ==
          xf(pos_map.at(sym_joint_names[i] + l_r_pair.second)));
      // velocities
      trajopt->AddLinearConstraint(
          x0(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.first + "dot")) ==
          xf(n_q + vel_map.at(sym_joint_names[i] + l_r_pair.second + "dot")));
      // inputs (ankle joint is not actuated)
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        trajopt->AddLinearConstraint(
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
    trajopt->AddBoundingBoxConstraint(VectorXd::Constant(n_u, -300),
                                      VectorXd::Constant(n_u, +300), ui);
  }

  // toe position constraint in y direction (avoid leg crossing)
  auto left_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_left", 1, 0.05, std::numeric_limits<double>::infinity());
  auto right_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
      &plant, "toe_right", 1, -std::numeric_limits<double>::infinity(), -0.05);
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

  // Testing -- constraint on initial floating base
  trajopt->AddConstraint(x0(0) == 1);

  // Testing -- constraint on the forces magnitude
  /*for (int i = 0; i < N; i++) {
    auto lambda = trajopt->force(0, i);
    trajopt->AddLinearConstraint(lambda(2) <= 700);
    trajopt->AddLinearConstraint(lambda(5) <= 700);
    trajopt->AddLinearConstraint(lambda(6) >= -1000);  // left leg four bar
    trajopt->AddLinearConstraint(lambda(6) <= 1000);   // left leg four bar
    trajopt->AddLinearConstraint(lambda(7) >= -500);   // left leg four bar
    trajopt->AddLinearConstraint(lambda(7) <= 500);    // left leg four bar
  }*/
  /*for (int i = 0; i < N - 1; i++) {
    auto lambda = trajopt->collocation_force(0, i);
    trajopt->AddLinearConstraint(lambda(2) <= 700);
    trajopt->AddLinearConstraint(lambda(5) <= 700);
    trajopt->AddLinearConstraint(lambda(6) >= -1000);  // left leg four bar
    trajopt->AddLinearConstraint(lambda(6) <= 1000);   // left leg four bar
    trajopt->AddLinearConstraint(lambda(7) >= -500);   // left leg four bar
    trajopt->AddLinearConstraint(lambda(7) <= 500);    // left leg four bar
  }*/
  // Testing -- constraint on normal force
  for (int i = 0; i < N; i++) {
    auto lambda = trajopt->force(0, i);
    trajopt->AddLinearConstraint(lambda(2) >= 10);
    trajopt->AddLinearConstraint(lambda(5) >= 10);
  }
  // Testing -- constraint on u
  /*for (int i = 0; i < N; i++) {
    auto ui = trajopt->input(i);
    trajopt->AddLinearConstraint(ui(6) >= 0);
  }*/

  // Scale decision variable
  double s_q_toe = 1;
  double s_v_toe_l = 1;
  double s_v_toe_r = 1;
  if (FLAGS_is_scale_variable) {
    // time
    trajopt->ScaleTimeVariables(0.008);
    // state
    // TODO: try increase the toe position by a factor of 10
    trajopt->ScaleStateVariables(0.5, 0, 3);
    if (s_q_toe > 1) {
      trajopt->ScaleStateVariables(s_q_toe, n_q - 2, n_q - 1);
    }
    trajopt->ScaleStateVariables(10, n_q, n_q + n_v - 3);
    trajopt->ScaleStateVariables(10 * s_v_toe_l, n_q + n_v - 2, n_q + n_v - 2);
    trajopt->ScaleStateVariables(10 * s_v_toe_r, n_q + n_v - 1, n_q + n_v - 1);
    // input
    trajopt->ScaleInputVariables(100, 0, 9);
    // force
    // TODO: try increase lambda 7 and 8 by 3 times
    trajopt->ScaleForceVariables(
        1000, 0, 0, ls_dataset.countConstraintsWithoutSkipping() - 1);
    if (!is_disable_kin_constraint_at_last_node) {
      trajopt->ScaleForceVariables(
          1000, 1, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);
    }
    // impulse
    // TODO: try increase impulse 7 and 8 by 2 times
    trajopt->ScaleImpulseVariables(
        10, 0, 0, rs_dataset.countConstraintsWithoutSkipping() - 1);  // 0.1
    // quaternion slack
    trajopt->ScaleQuaternionSlackVariables(30);
    // Constraint slack
    trajopt->ScaleKinConstraintSlackVariables(50, 0, 0, 5);
    trajopt->ScaleKinConstraintSlackVariables(500, 0, 6, 7);
  }

  // add cost
  //  const MatrixXd Q = MatrixXd::Zero(n_v, n_v);
  //  const MatrixXd R = MatrixXd::Zero(n_u, n_u);
  MatrixXd Q = 5 * 0.1 * MatrixXd::Identity(n_v, n_v);
  MatrixXd R = 0.1 * 0.01 * MatrixXd::Identity(n_u, n_u);
  Q(n_v - 2, n_v - 2) /= (s_v_toe_l * s_v_toe_l);
  Q(n_v - 1, n_v - 1) /= (s_v_toe_r * s_v_toe_r);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);
  /*// Add cost without time
  for (int i = 0; i < N - 1; i++) {
    auto v0 = trajopt->state(i).tail(n_v);
    auto v1 = trajopt->state(i + 1).tail(n_v);
    auto h = 0.4 / 15.0;
    trajopt->AddCost(((v0.transpose() * Q * v0) * h / 2)(0));
    trajopt->AddCost(((v1.transpose() * Q * v1) * h / 2)(0));
  }
  for (int i = 0; i < N - 1; i++) {
    auto u0 = trajopt->input(i);
    auto u1 = trajopt->input(i + 1);
    auto h = 0.4 / 15.0;
    trajopt->AddCost(((u0.transpose() * R * u0) * h / 2)(0));
    trajopt->AddCost(((u1.transpose() * R * u1) * h / 2)(0));
  }*/

  // Other costs
  double Q_lamb_diff = 0;//0.000001;
  double Q_v_diff_double = 0;//0.01 * 5;
  double Q_u_diff = 0;//0.00001;
  double Q_q_hip_roll = 1;
  double Q_q_hip_yaw = 1;
  double Q_q_quat_xyz = 0;

  // add cost on force difference wrt time
  bool diff_with_force_at_collocation = false;
  if (Q_lamb_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto lambda0 = trajopt->force(0, i);
      auto lambda1 = trajopt->force(0, i + 1);
      auto lambdac = trajopt->collocation_force(0, i);
      if (diff_with_force_at_collocation) {
        trajopt->AddCost(Q_lamb_diff *
                         (lambda0 - lambdac).dot(lambda0 - lambdac));
        trajopt->AddCost(Q_lamb_diff *
                         (lambdac - lambda1).dot(lambdac - lambda1));
      } else {
        trajopt->AddCost(Q_lamb_diff *
                         (lambda0 - lambda1).dot(lambda0 - lambda1));
      }
    }
  }
  // add cost on vel difference wrt time
  MatrixXd Q_v_diff = Q_v_diff_double * MatrixXd::Identity(n_v, n_v);
  Q_v_diff(n_v - 2, n_v - 2) /= (s_v_toe_l * s_v_toe_l);
  Q_v_diff(n_v - 1, n_v - 1) /= (s_v_toe_r * s_v_toe_r);
  if (Q_v_diff_double) {
    for (int i = 0; i < N - 1; i++) {
      auto v0 = trajopt->state(i).tail(n_v);
      auto v1 = trajopt->state(i + 1).tail(n_v);
      trajopt->AddCost((v0 - v1).dot(Q_v_diff * (v0 - v1)));
    }
  }
  // add cost on input difference wrt time
  if (Q_u_diff) {
    for (int i = 0; i < N - 1; i++) {
      auto u0 = trajopt->input(i);
      auto u1 = trajopt->input(i + 1);
      trajopt->AddCost(Q_u_diff * (u0 - u1).dot(u0 - u1));
    }
  }
  // add cost on joint position
  if (Q_q_hip_roll) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(7, 2);
      trajopt->AddCost(Q_q_hip_roll * q.transpose() * q);
    }
  }
  if (Q_q_hip_yaw) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(9, 2);
      trajopt->AddCost(Q_q_hip_yaw * q.transpose() * q);
    }
  }
  if (Q_q_quat_xyz) {
    for (int i = 0; i < N; i++) {
      auto q = trajopt->state(i).segment(1, 3);
      trajopt->AddCost(Q_q_quat_xyz * q.transpose() * q);
    }
  }

  // initial guess
  if (!init_file.empty()) {
    MatrixXd z0 = readCSV(data_directory + init_file);
    trajopt->SetInitialGuessForAllVariables(z0);
  } else {
    // Add random initial guess first (the seed for RNG is fixed)
    trajopt->SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt->decision_variables().size()));

    // Do inverse kinematics to get q initial guess
    vector<VectorXd> q_seed = GetInitGuessForQ(N, stride_length, plant);
    // Do finite differencing to get v initial guess
    vector<VectorXd> v_seed =
        GetInitGuessForV(q_seed, duration / (N - 1), plant);
    for (int i = 0; i < N; i++) {
      auto xi = trajopt->state(i);
      VectorXd xi_seed(n_q + n_v);
      xi_seed << q_seed.at(i), v_seed.at(i);
      trajopt->SetInitialGuess(xi, xi_seed);
    }

    /*// Testing -- initial guess for input
    // These guesses are from a good solution
    for (int i = 0; i < N; i++) {
      auto u = trajopt->input(i);
      trajopt->SetInitialGuess(u(0), 20);
      trajopt->SetInitialGuess(u(2), -30);
      trajopt->SetInitialGuess(u(4), 30);
      trajopt->SetInitialGuess(u(6), 50);
      trajopt->SetInitialGuess(u(8), 20);
    }
    // Testing -- initial guess for force (also forces at collocation)
    for (int i = 0; i < N; i++) {
      auto lambda = trajopt->force(0, i);
      //      trajopt->SetInitialGuess(lambda(0), 0);
      //      trajopt->SetInitialGuess(lambda(1), 0);
      trajopt->SetInitialGuess(lambda(2), 170);
      //      trajopt->SetInitialGuess(lambda(3), 0);
      //      trajopt->SetInitialGuess(lambda(4), 0);
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
    // Testing -- initial guess for slack
    auto vars_kinematics = trajopt->collocation_slack_vars(0);
    for (int i = 0; i < vars_kinematics.size(); i++) {
      trajopt->SetInitialGuess(vars_kinematics(i), 0);
    }
    auto vars_quaternion = trajopt->quaternion_slack_vars(0);
    for (int i = 0; i < vars_quaternion.size(); i++) {
      trajopt->SetInitialGuess(vars_quaternion(i), 0);
    }
    // Testing -- initial condition for timestep
    for (int i = 0; i < N - 1; i++) {
      auto h_var = trajopt->timestep(i);
      trajopt->SetInitialGuess(h_var(0), duration / (N - 1));
    }
    // Testing -- initial condition for post impact velocity
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
    }  // end for (l_r_pairs)*/
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

  // Testing - check if the nonlinear constraints are all satisfied
  // bool constraint_satisfied = solvers::CheckGenericConstraints(*trajopt,
  //                             result, tol);
  // cout << "constraint_satisfied = " << constraint_satisfied << endl;

  // store the solution of the decision variable
  VectorXd z = result.GetSolution(trajopt->decision_variables());
  if (to_store_data) {
    writeCSV(data_directory + string("z.csv"), z);
  }
  for (int i = 0; i < z.size(); i++) {
    cout << i << ": " << trajopt->decision_variables()[i] << ", " << z[i]
         << endl;
  }
  cout << endl;

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

  // Calculate each term of the cost
  double total_cost = 0;
  double cost_x = 0;
  for (int i = 0; i < N - 1; i++) {
    auto v0 = state_at_knots.col(i).tail(n_v);
    auto v1 = state_at_knots.col(i + 1).tail(n_v);
    auto h = time_at_knots(i + 1) - time_at_knots(i);
    cost_x += ((v0.transpose() * Q * v0) * h / 2)(0);
    cost_x += ((v1.transpose() * Q * v1) * h / 2)(0);
  }
  total_cost += cost_x;
  cout << "cost_x = " << cost_x << endl;
  double cost_u = 0;
  for (int i = 0; i < N - 1; i++) {
    auto u0 = input_at_knots.col(i);
    auto u1 = input_at_knots.col(i + 1);
    auto h = time_at_knots(i + 1) - time_at_knots(i);
    cost_u += ((u0.transpose() * R * u0) * h / 2)(0);
    cost_u += ((u1.transpose() * R * u1) * h / 2)(0);
  }
  total_cost += cost_u;
  cout << "cost_u = " << cost_u << endl;
  double cost_lambda = 0;
  for (int i = 0; i < num_time_samples.size(); i++) {
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
          Q_lamb_diff * (lambda0 - lambdac).dot(lambda0 - lambdac);
      cost_lambda_diff +=
          Q_lamb_diff * (lambdac - lambda1).dot(lambdac - lambda1);
    } else {
      cost_lambda_diff +=
          Q_lamb_diff * (lambda0 - lambda1).dot(lambda0 - lambda1);
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
    cost_u_diff += Q_u_diff * (u0 - u1).dot(u0 - u1);
  }
  total_cost += cost_u_diff;
  cout << "cost_u_diff = " << cost_u_diff << endl;
  // add cost on joint position
  double cost_q_hip_roll = 0;
  for (int i = 0; i < N; i++) {
    auto q = result.GetSolution(trajopt->state(i).segment(7, 2));
    cost_q_hip_roll += Q_q_hip_roll * q.transpose() * q;
  }
  total_cost += cost_q_hip_roll;
  cout << "cost_q_hip_roll = " << cost_q_hip_roll << endl;
  // add cost on quaternion
  double cost_q_quat_xyz = 0;
  for (int i = 0; i < N; i++) {
    auto q = result.GetSolution(trajopt->state(i).segment(1, 3));
    cost_q_quat_xyz += Q_q_quat_xyz * q.transpose() * q;
  }
  total_cost += cost_q_quat_xyz;
  cout << "cost_q_quat_xyz = " << cost_q_quat_xyz << endl;

  cout << "total_cost = " << total_cost << endl;

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
                  FLAGS_init_file, FLAGS_tol, FLAGS_store_data,
                  FLAGS_scale_option);
}
