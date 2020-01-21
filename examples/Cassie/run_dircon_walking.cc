#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>

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

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::VectorX;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;

using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using drake::multibody::BodyIndex;
using drake::multibody::JointActuator;
using drake::multibody::JointActuatorIndex;
using drake::multibody::ModelInstanceIndex;

using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;

using dairlib::systems::trajectory_optimization::DirconAbstractConstraint;
using dairlib::systems::trajectory_optimization::DirconDynamicConstraint;
using dairlib::systems::trajectory_optimization::DirconKinConstraintType;
using dairlib::systems::trajectory_optimization::DirconKinematicConstraint;
using dairlib::systems::trajectory_optimization::DirconOptions;
using dairlib::systems::trajectory_optimization::HybridDircon;

using dairlib::systems::SubvectorPassThrough;

using dairlib::goldilocks_models::readCSV;
using dairlib::goldilocks_models::writeCSV;

using dairlib::multibody::ContactInfo;
using dairlib::multibody::FixedPointSolver;
using dairlib::multibody::GetBodyIndexFromName;

DEFINE_string(init_file, "", "the file name of initial guess");
DEFINE_string(data_directory, "../dairlib_data/cassie_trajopt_data/",
              "directory to save/read data");
DEFINE_bool(store_data, false, "To store solution or not");
DEFINE_int32(max_iter, 100000, "Iteration limit");
DEFINE_double(duration, 0.4, "Duration of the single support phase (s)");
DEFINE_double(tol, 1e-4, "Tolerance for constraint violation and dual gap");

// Parameters which enable dircon-improving features
DEFINE_bool(is_scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(is_scale_variable, true, "Scale the decision variable");

namespace dairlib {

/// Trajectory optimization of fixed-spring cassie squatting
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
      MultibodyPlant<double> plant_ik;
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
      simulator.StepTo(1.0 / N);
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
            string init_file, double tol, bool to_store_data) {
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant;
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

  std::vector<DirconOptions> options_list;
  options_list.push_back(DirconOptions(ls_dataset.countConstraints(), &plant));
  options_list.push_back(DirconOptions(rs_dataset.countConstraints(), &plant));
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
      options_list[i].setImpConstraintScaling(1.0 / 12.0, 3, n_v - 1);
    }
  }

  // Testing - check if the above is set correctly:
  auto scale_map = options_list[0].getDynConstraintScaling();
  for (auto member : scale_map) {
    cout << member.first << ", " << member.second << endl;
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
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->state_vars_by_mode(
      num_time_samples.size() - 1,
      num_time_samples[num_time_samples.size() - 1] - 1);

  // parameters
  // double duration = 0.4;
  double stride_length = 0.2;
  double ground_incline = 0;

  // Fix time duration
  //  trajopt->AddDurationBounds(duration, duration);

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

  // Floating base periodicity
  trajopt->AddLinearConstraint(x0(pos_map.at("position[0]")) ==
                               xf(pos_map.at("position[0]")));
  trajopt->AddLinearConstraint(x0(pos_map.at("position[1]")) ==
                               -xf(pos_map.at("position[1]")));
  trajopt->AddLinearConstraint(x0(pos_map.at("position[2]")) ==
                               xf(pos_map.at("position[2]")));
  trajopt->AddLinearConstraint(x0(pos_map.at("position[3]")) ==
                               -xf(pos_map.at("position[3]")));
  trajopt->AddLinearConstraint(x0(pos_map.at("position[5]")) ==
                               -xf(pos_map.at("position[5]")));
  if (ground_incline == 0) {
    trajopt->AddLinearConstraint(x0(pos_map.at("position[6]")) ==
                                 xf(pos_map.at("position[6]")));
  }
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("velocity[0]")) ==
                               xf(n_q + vel_map.at("velocity[0]")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("velocity[1]")) ==
                               -xf(n_q + vel_map.at("velocity[1]")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("velocity[2]")) ==
                               xf(n_q + vel_map.at("velocity[2]")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("velocity[3]")) ==
                               xf(n_q + vel_map.at("velocity[3]")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("velocity[4]")) ==
                               -xf(n_q + vel_map.at("velocity[4]")));
  trajopt->AddLinearConstraint(x0(n_q + vel_map.at("velocity[5]")) ==
                               xf(n_q + vel_map.at("velocity[5]")));

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
      // inputs
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
  /*auto left_foot_constraint = std::make_shared<OneDimBodyPosConstraint>(
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
  }*/

  // add cost
  const MatrixXd Q = 10 * 12.5 * MatrixXd::Identity(n_v, n_v);
  const MatrixXd R = 12.5 * MatrixXd::Identity(n_u, n_u);
  trajopt->AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt->AddRunningCost(u.transpose() * R * u);

  // TODO(yminchen): add variable scaling and review the code of initial guess
  // generation

  // Testing
  if (FLAGS_is_scale_variable) {
    // time
    trajopt->ScaleTimeVariables(0.015);
    // state
    trajopt->ScaleStateVariables(6, n_q, n_q + 9);
    trajopt->ScaleStateVariables(3, n_q + 10, n_q + n_v - 1);
    // input
    trajopt->ScaleInputVariables(60, 0, 1);
    trajopt->ScaleInputVariables(300, 2, 3);  // 300
    trajopt->ScaleInputVariables(60, 4, 7);
    trajopt->ScaleInputVariables(600, 8, 9);  // 600
    // force
    trajopt->ScaleForceVariables(10, 0, 0, 1);
    trajopt->ScaleForceVariables(1000, 0, 2, 2);  // 1000
    trajopt->ScaleForceVariables(10, 0, 3, 4);
    trajopt->ScaleForceVariables(1000, 0, 5, 5);
    trajopt->ScaleForceVariables(10, 0, 6, 7);
    trajopt->ScaleForceVariables(1000, 0, 8, 8);
    trajopt->ScaleForceVariables(10, 0, 9, 10);
    trajopt->ScaleForceVariables(1000, 0, 11, 11);
    trajopt->ScaleForceVariables(600, 0, 12, 13);
    //    trajopt->ScaleForceVariables(
    //        600, 0, 0, ls_dataset.countConstraintsWithoutSkipping() - 1);
    //    trajopt->ScaleQuaternionSlackVariables(0.5);
    //    trajopt->ScaleKinConstraintSlackVariables(0.1);

    for (int i = 0; i < trajopt->decision_variables().size(); i++) {
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
    vector<VectorXd> q_seed = GetInitGuessForQ(N_ss, stride_length, plant);
    // Do finite differencing to get v initial guess
    vector<VectorXd> v_seed =
        GetInitGuessForV(q_seed, duration / (N_ss - 1), plant);
    for (int i = 0; i < N; i++) {
      auto xi = trajopt->state(i);
      VectorXd xi_seed(n_q + n_v);
      if (i < N_ss) {
        xi_seed << q_seed.at(i).head(4) / quaternion_scale,
            q_seed.at(i).tail(n_q - 4), v_seed.at(i) / omega_scale;
      } else {
        xi_seed << q_seed.at(N_ss - 1).head(4) / quaternion_scale,
            q_seed.at(N_ss - 1).tail(n_q - 4),
            v_seed.at(N_ss - 1) / omega_scale;
      }
      trajopt->SetInitialGuess(xi, xi_seed);
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

  // Testing - check if the nonlinear constraints are all satisfied
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
