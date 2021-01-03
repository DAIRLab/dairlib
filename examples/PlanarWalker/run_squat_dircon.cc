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

DEFINE_double(strideLength, 0.1, "The stride length.");
DEFINE_double(duration, 1, "The stride duration");
DEFINE_bool(autodiff, false, "Double or autodiff version");

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

  int num_knotpoints = 20;
  double min_T = .1;
  double max_T = 3;
  
  auto double_support = DirconMode<T>(evaluators, num_knotpoints,
      min_T, max_T);
  double_support.MakeConstraintRelative(0, 0);  // x-coordinate
  double_support.MakeConstraintRelative(1, 0);  // x-coordinate


  auto sequence = DirconModeSequence<T>(plant);
  sequence.AddMode(&double_support);
  auto trajopt = Dircon<T>(sequence);

  trajopt.AddDurationBounds(duration, duration);

  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "/home/shane/snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 20000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 100000);

  for (int j = 0; j < sequence.num_modes(); j++) {
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j],
                                      init_vc_traj[j]);
  }

  int n_v = plant.num_velocities();

  // Constraints
  auto u = trajopt.input();
  auto x = trajopt.state();
  auto x0 = trajopt.initial_state();
  auto xf = trajopt.final_state();
  auto xmid = trajopt.state_vars(0, (num_knotpoints - 1) / 2);

  // Initial height
  trajopt.AddBoundingBoxConstraint(0.8, 0.8, x0(positions_map.at("planar_z")));
  // Final height
  //trajopt.AddBoundingBoxConstraint(0.8, 0.8, xf(positions_map.at("planar_z")));
  // Bottom height
  trajopt.AddBoundingBoxConstraint(0.6, 0.6, xf(positions_map.at("planar_z")));

  // Initial and final rotation of "torso""
  trajopt.AddLinearConstraint( x0(positions_map.at("right_knee_pin")) ==  -x0(positions_map.at("left_knee_pin")));
  //trajopt.AddLinearConstraint( xmid(positions_map.at("right_knee_pin")) ==  -xmid(positions_map.at("left_knee_pin")));
  trajopt.AddLinearConstraint( xf(positions_map.at("right_knee_pin")) ==  -xf(positions_map.at("left_knee_pin")));

  // Fore-aft position
  trajopt.AddLinearConstraint(x0(positions_map["planar_x"]) == 0);
  //trajopt.AddLinearConstraint(xmid(positions_map["planar_x"]) == 0);
  trajopt.AddLinearConstraint(xf(positions_map["planar_x"]) == 0);

  // start/end velocity constraints
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   xf.tail(n_v));


  // Not sure what this is doing
  for (int i = 0; i < num_knotpoints; i++) {
    trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(0, i)(1));
  }

  const double R = 10;  // Cost on input effort
  const MatrixXd Q = 10  * MatrixXd::Identity(n_v, n_v); // Cost on velocity
  trajopt.AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt.AddRunningCost(u.transpose()*R*u);

  std::vector<unsigned int> visualizer_poses;
  visualizer_poses.push_back(3);

  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf"),
      visualizer_poses, 0.2, "base");

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

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
    sleep(5);
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

  plant->Finalize();
  plant_vis->Finalize();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant->num_positions() +
                       plant->num_velocities());

  Eigen::VectorXd init_l_vec(3);
  init_l_vec << 0, 0, 20*9.81;
  int nu = plant->num_actuators();
  int nx = plant->num_positions() + plant->num_velocities();
  int N = 10;

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  // Initialize state trajectory
  std::vector<double> init_time;
  for (int i = 0; i < 2*N-1; i++) {
    init_time.push_back(i*.2);
    init_x.push_back(x0 + .1*VectorXd::Random(nx));
    init_u.push_back(VectorXd::Random(nu));
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
      init_time_j.push_back(i*.2);
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
