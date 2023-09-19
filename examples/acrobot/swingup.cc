#include <memory>
#include <iostream>
#include <chrono>

#include <gflags/gflags.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"

DEFINE_double(duration, 5, "The trajectory duration");
DEFINE_double(u_max, 100, "Max effort");
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
    double duration,
    double u_max,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T>& plant = *plant_ptr;
  SceneGraph<double>& scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));

  int num_knotpoints = 30;
  auto evaluators = multibody::KinematicEvaluatorSet<T>(plant);
  auto mode = DirconMode<T>(evaluators, num_knotpoints, duration, duration);
  auto sequence = DirconModeSequence<T>(plant);
  sequence.AddMode(&mode);
  auto trajopt = Dircon<T>(sequence);
  auto& prog = trajopt.prog();

  trajopt.AddDurationBounds(duration, duration);

  prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 2000);

  trajopt.drake::systems::trajectory_optimization::MultipleShooting::
      SetInitialTrajectory(init_u_traj, init_x_traj);

  ///
  /// PROBLEM SETUP
  ///
  auto x0 = trajopt.initial_state();
  auto xf = trajopt.final_state();

  VectorXd x0_vec(4), xf_vec(4);
  x0_vec << 0, 0, 0, 0;
  xf_vec << M_PI, 0, 0, 0;

  int nq = plant.num_positions();
  prog.AddBoundingBoxConstraint(x0_vec, x0_vec,x0);
  prog.AddBoundingBoxConstraint(xf_vec, xf_vec, xf);

  const double R = 1;  // Cost on input effort
  auto u = trajopt.input();
  trajopt.AddRunningCost(u.transpose()*R*u);

  trajopt.AddConstraintToAllKnotPoints(u[0] <= u_max);
  trajopt.AddConstraintToAllKnotPoints(u[0] >= -u_max);
  ///
  /// END PROBLEM SETUP
  ///

  int visualizer_poses = 30;
  double alpha = .2;

  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/acrobot/Acrobot.urdf"),
      visualizer_poses, alpha);

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(prog, prog.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  if (result.is_success()) {
    std::cout << "Success!" <<std::endl;
  } else {
    std::cout << "Failure!" <<std::endl;
  }

  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  // visualizer
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::ConnectTrajectoryVisualizer(plant_double_ptr, &builder,
                                         &scene_graph, pp_xtraj);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
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
      dairlib::FindResourceOrThrow("examples/acrobot/Acrobot.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);

  plant->Finalize();
  plant_vis->Finalize();

  Eigen::VectorXd x0(4), xf(4);
  x0 << 0, 0, 0, 0;
  xf << M_PI, 0, 0, 0;

  int nu = plant->num_actuators();
  int nx = plant->num_positions() + plant->num_velocities();
  int N = 10;

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;

  // Initialize state trajectory
  std::vector<double> init_time;
  for (int i = 0; i < N; i++) {
    init_time.push_back(i*.2);
    init_x.push_back(x0*(N - i - 1.0)/(N - 1.0) + xf * i/(N - 1.0));
    init_u.push_back(5*VectorXd::Random(nu));
  }
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

  dairlib::runDircon<double>(
    std::move(plant), plant_vis.get(), std::move(scene_graph),
    FLAGS_duration, FLAGS_u_max, init_x_traj, init_u_traj);
}
