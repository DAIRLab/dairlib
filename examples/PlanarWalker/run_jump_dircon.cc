#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "multibody/kinematic/distance_evaluator.h"

DEFINE_double(strideLength, 0.1, "The stride length.");
DEFINE_double(duration, 1, "The squat duration");
DEFINE_double(min_duration, 0.6, "The squat duration");
DEFINE_double(max_duration, 5, "The squat duration");

DEFINE_bool(autodiff, false, "Double or autodiff version");
DEFINE_double(bottomHeight, 0.8, "The bottom height.");
DEFINE_double(topHeight, 1.1, "The top height.");

using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace dairlib {
namespace {

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;

template <typename T>
void runDircon(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    double stride_length,
    double duration,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T>& plant = *plant_ptr;
  SceneGraph<double>& scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  for (auto const& element : positions_map)
    cout << element.first << " = " << element.second << endl;
  for (auto const& element : velocities_map)
    cout << element.first << " = " << element.second << endl;

  const auto& left_lower_leg = plant.GetFrameByName("left_lower_leg");
  const auto& right_lower_leg = plant.GetFrameByName("right_lower_leg");

  Vector3d pt(0, 0, -.5);
  double mu = 1;

  auto left_foot_eval = multibody::WorldPointEvaluator<T>(plant, pt,
      left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});
  left_foot_eval.set_frictional();
  left_foot_eval.set_mu(mu);

  auto right_foot_eval = multibody::WorldPointEvaluator<T>(plant, pt,
      right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), {0, 2});
  right_foot_eval.set_frictional();
  right_foot_eval.set_mu(mu);

  auto evaluators= multibody::KinematicEvaluatorSet<T>(plant);
  evaluators.add_evaluator(&left_foot_eval);
  evaluators.add_evaluator(&right_foot_eval);

  int num_knotpoints = 10;
  double min_T = .03;
  double max_T = 3;
  
  auto double_support = DirconMode<T>(evaluators, num_knotpoints,
      min_T, max_T);
  double_support.MakeConstraintRelative(0, 0);  // x-coordinate
  double_support.MakeConstraintRelative(1, 0);  // x-coordinate


  auto evaluators_flight = multibody::KinematicEvaluatorSet<T>(plant);
  auto flight_mode = DirconMode<T>(evaluators_flight, num_knotpoints,
                                      min_T, max_T);

  auto sequence = DirconModeSequence<T>(plant);
  //sequence.AddMode(&double_support);
  sequence.AddMode(&flight_mode);
  sequence.AddMode(&double_support);
  auto trajopt = Dircon<T>(sequence);

  trajopt.AddDurationBounds(FLAGS_min_duration, FLAGS_max_duration);

  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "/home/shane/snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 20000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 100000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           1e-4);  // target nonlinear constraint violation

  for (int j = 0; j < sequence.num_modes(); j++) {
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j%1], init_lc_traj[j%1],
                                      init_vc_traj[j%1]);
  }

  int n_v = plant.num_velocities();
  int n_q = plant.num_positions();
  int n_u = plant.num_actuators();
  std::cout << n_u << std::endl;

  // Constraints
  auto u = trajopt.input();
  auto x = trajopt.state();
  auto x0 = trajopt.initial_state();
  auto xf = trajopt.final_state();
  auto xmid = trajopt.state_vars(0, (num_knotpoints - 1) / 2);

  // Initial height
  trajopt.AddBoundingBoxConstraint(FLAGS_topHeight, FLAGS_topHeight, x0(positions_map.at("planar_z")));
  // Final height
  //trajopt.AddBoundingBoxConstraint(FLAGS_topHeight, FLAGS_topHeight, xmid(positions_map.at("planar_z")));
  // Bottom height
  trajopt.AddBoundingBoxConstraint(FLAGS_bottomHeight, FLAGS_bottomHeight, xf(positions_map.at("planar_z")));

  // Initial and final rotation of "torso""
  //trajopt.AddLinearConstraint( x0(positions_map.at("right_knee_pin")) ==  -x0(positions_map.at("left_knee_pin")));
  //trajopt.AddLinearConstraint( xmid(positions_map.at("right_knee_pin")) ==  -xmid(positions_map.at("left_knee_pin")));
  //trajopt.AddLinearConstraint( xf(positions_map.at("right_knee_pin")) ==  -xf(positions_map.at("left_knee_pin")));

  // Fore-aft position
  trajopt.AddLinearConstraint(x0(positions_map["planar_x"]) == 0);
  //trajopt.AddLinearConstraint(xmid(positions_map["planar_x"]) == 0);
  trajopt.AddLinearConstraint(xf(positions_map["planar_x"]) == 0);

  // start/end velocity constraints
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                  x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                  xf.tail(n_v));
  

  // Set foot distances
  std::vector<int> x_active({0});
  auto distance_foot_distance_eval = multibody::DistanceEvaluator<T>(
      plant, pt, right_lower_leg, pt, left_lower_leg, 0);
  /*
  auto left_foot_x_eval = multibody::WorldPointEvaluator<T>(plant, pt,
                                                          left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), x_active);
  auto right_foot_x_eval = multibody::WorldPointEvaluator<T>(plant, pt,
                                                           right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), x_active);
  auto foot_x_evaluators = multibody::KinematicEvaluatorSet<T>(plant);
  foot_x_evaluators.add_evaluator(&right_foot_x_eval);
  foot_x_evaluators.add_evaluator(&left_foot_x_eval);

  auto foot_x_lb =
      Eigen::Vector2d(0.05, -0.15);
  auto foot_x_ub =
      Eigen::Vector2d(0.15, -0.05);
    */
  auto foot_distance_evalutors = multibody::KinematicEvaluatorSet<T>(plant);
  foot_distance_evalutors.add_evaluator(&distance_foot_distance_eval);

  auto foot_distance_lb =
      Eigen::VectorXd(1);
  auto foot_distance_ub =
      Eigen::VectorXd(1);
  foot_distance_lb(0) = 0.1;
  foot_distance_ub(0) = 0.3;

  auto foot_x_constraint =
      std::make_shared<multibody::KinematicPositionConstraint<T>>(
          plant, foot_distance_evalutors, foot_distance_lb, foot_distance_ub);

  std::vector<int> z_active({2});
  auto left_foot_z_eval = multibody::WorldPointEvaluator<T>(plant, pt,
                                                            left_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), z_active);
  auto right_foot_z_eval = multibody::WorldPointEvaluator<T>(plant, pt,
                                                             right_lower_leg, Matrix3d::Identity(), Vector3d::Zero(), z_active);
  auto foot_z_evaluators = multibody::KinematicEvaluatorSet<T>(plant);
  foot_z_evaluators.add_evaluator(&left_foot_z_eval);
  foot_z_evaluators.add_evaluator(&right_foot_z_eval);
  auto foot_z_lb =
      Eigen::Vector2d(0, 0);
  auto foot_z_ub =
      Eigen::Vector2d(10, 10);
  auto foot_z_constraint =
      std::make_shared<multibody::KinematicPositionConstraint<T>>(
          plant, foot_z_evaluators, foot_z_lb, foot_z_ub);

  for (int index = 0; index < num_knotpoints; index++) {
    auto x_local = trajopt.state(index);
    trajopt.AddConstraint(foot_x_constraint, x_local.head(n_q));
    //trajopt.AddConstraint(foot_z_constraint, x_local.head(n_q));

  }

  const double R = 10;  // Cost on input effort
  const MatrixXd Q = 5  * MatrixXd::Identity(n_v, n_v); // Cost on velocity
  //trajopt.AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt.AddRunningCost(u.transpose()*R*u);

  std::vector<unsigned int> visualizer_poses;
  visualizer_poses.push_back(3);
  visualizer_poses.push_back(3);
  //visualizer_poses.push_back(3);

  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf"),
      visualizer_poses, 0.2, "base");

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  // visualizer
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(plant_double_ptr,
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.25);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}
}  // namespace
}  // namespace dairlib


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);

  plant->WeldFrames(
      plant->world_frame(), plant->GetFrameByName("base"),
      drake::math::RigidTransform<double>());
  plant_vis->WeldFrames(
      plant_vis->world_frame(), plant_vis->GetFrameByName("base"),
      drake::math::RigidTransform<double>());
  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant->num_positions() +
                       plant->num_velocities());

  Eigen::VectorXd init_l_vec(3);
  init_l_vec << 0, 0, 20*9.81;
  int nu = plant->num_actuators();
  int nx = plant->num_positions() + plant->num_velocities();
  int nq = plant->num_positions();
  int N = 20;

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  // Initialize state trajectory
  std::vector<double> init_time;
  VectorXd xState(nx);
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(*plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plant);

  xState(nq + velocities_map.at("hip_pindot")) = -0.1;
  xState(nq + velocities_map.at("planar_rotydot")) = 0.1;
  xState(nq + velocities_map.at("planar_xdot")) = 0;
  xState(nq + velocities_map.at("planar_zdot")) = -(FLAGS_topHeight-FLAGS_bottomHeight)/FLAGS_duration/2;
  xState(nq + velocities_map.at("left_knee_pindot")) = -0.1;
  xState(nq + velocities_map.at("right_knee_pindot")) = 0.1;
  double time = 0;
  for (int i = 0; i < N/2; i++) {
    time=i*FLAGS_duration/(N-1)/2;
    init_time.push_back(time);

    xState(positions_map.at("hip_pin")) = -0.4 + xState(nq + velocities_map.at("hip_pindot")) * time;
    xState(positions_map.at("planar_roty")) = 0.4 + xState(nq + velocities_map.at("planar_rotydot")) * time;
    xState(positions_map.at("planar_x")) = 0;
    xState(positions_map.at("planar_z")) = FLAGS_topHeight + xState(nq + velocities_map.at("planar_zdot")) * time;
    xState(positions_map.at("left_knee_pin")) = -0.3 + xState(nq + velocities_map.at("left_knee_pindot")) * time;
    xState(positions_map.at("right_knee_pin")) = 0.3 + xState(nq + velocities_map.at("right_knee_pindot")) * time;
    init_x.push_back(xState);
    init_u.push_back(VectorXd::Zero(nu));
  }

  xState = -xState;
  for (int i = N/2; i < N; i++) {
    time=i*FLAGS_duration/(N-1);
    init_time.push_back(time);

    xState(positions_map.at("hip_pin")) = -0.4 + xState(nq + velocities_map.at("hip_pindot")) * time;
    xState(positions_map.at("planar_roty")) = 0.4 + xState(nq + velocities_map.at("planar_rotydot")) * time;
    xState(positions_map.at("planar_x")) = 0;
    xState(positions_map.at("planar_z")) = FLAGS_topHeight + xState(nq + velocities_map.at("planar_zdot")) * time;
    xState(positions_map.at("left_knee_pin")) = -0.3 + xState(nq + velocities_map.at("left_knee_pindot")) * time;
    xState(positions_map.at("right_knee_pin")) = 0.3 + xState(nq + velocities_map.at("right_knee_pindot")) * time;
    init_x.push_back(xState);
    init_u.push_back(VectorXd::Zero(nu));
  }


  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

  //Initialize force trajectories
  for (int j = 0; j < 2; j++) {    
    std::vector<MatrixXd> init_l_j;
    std::vector<MatrixXd> init_lc_j;
    std::vector<MatrixXd> init_vc_j;
    std::vector<double> init_time_j;
    for (int i = 0; i < N; i++) {
      init_time_j.push_back(i*FLAGS_duration/(N-1));
      init_l_j.push_back(init_l_vec);
      init_lc_j.push_back(init_l_vec);
      init_vc_j.push_back(VectorXd::Zero(3));
    }

    auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
    auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
    auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

    init_l_traj.push_back(init_l_traj_j);
    init_lc_traj.push_back(init_lc_traj_j);
    init_vc_traj.push_back(init_vc_traj_j);
  }

  if (FLAGS_autodiff) {
    std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
        drake::systems::System<double>::ToAutoDiffXd(*plant);
    dairlib::runDircon<drake::AutoDiffXd>(
      std::move(plant_autodiff), plant_vis.get(), std::move(scene_graph),
      FLAGS_strideLength, FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  } else {
    dairlib::runDircon<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph),
      FLAGS_strideLength, FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  }
}

