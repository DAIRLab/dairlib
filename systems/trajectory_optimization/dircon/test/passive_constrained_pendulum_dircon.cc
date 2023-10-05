#pragma GCC diagnostic ignored "-Wuninitialized"

#include <gflags/gflags.h>

#include <memory>
#include <chrono>
#include <iostream>

#include <gflags/gflags.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"

/// Simple example using WorldPointEvaluator.Starting from an acrobot, adds
/// two constraints.
///  1) A WorldPointEvaluator to simulate a ball joint between the base and
///      the world.
///  2) A DistanceEvaluator between the base and a point on the lower link.
///     This converts the system to a 3D passive pendulum, as the lower and 
///     upperlinks will be fixed relative to one another.
///
/// Runs DIRCON from a given initial condition.

DEFINE_bool(autodiff, false, "Use double or autodiff");

namespace dairlib {
namespace {
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;

// Fixed path to double pendulum SDF model.
static const char* const kDoublePendulumUrdfPath =
  "systems/trajectory_optimization/dircon/test/acrobot_floating.urdf";

template <typename T>
void runDircon() {
  const std::string urdf_path =
      FindResourceOrThrow(kDoublePendulumUrdfPath);
  MultibodyPlant<double> plant_double(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  drake::systems::DiagramBuilder<double> builder;
  auto& scene_graph = *builder.AddSystem<SceneGraph>();
  Parser parser(&plant_double);
  Parser parser_vis(&plant_vis, &scene_graph);

  parser.AddModelFromFile(urdf_path);
  plant_double.Finalize();

  parser_vis.AddModelFromFile(urdf_path);
  plant_vis.Finalize();

  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_pointer;
  if (typeid(T) == typeid(drake::AutoDiffXd)) {
    plant_pointer = drake::systems::System<double>::ToAutoDiffXd(
      plant_double);
  }
  const MultibodyPlant<T>& plant = typeid(T) == typeid(double) ? 
      reinterpret_cast<const MultibodyPlant<T>&>(plant_double) : 
      reinterpret_cast<const MultibodyPlant<T>&>(*plant_pointer);

  const auto& base = plant.GetFrameByName("base_link");
  const auto& lower_link = plant.GetFrameByName("lower_link");

  Vector3d pt1 = Vector3d::Zero();
  Vector3d pt2(-1, 0, 0);
  double distance = .7;

  auto distance_eval = multibody::DistanceEvaluator<T>(plant, pt1, base,
      pt2, lower_link, distance);

  auto pin_eval = multibody::WorldPointEvaluator<T>(plant,
      Vector3d::Zero(), base);

  auto evaluators = multibody::KinematicEvaluatorSet<T>(plant);
  evaluators.add_evaluator(&distance_eval);
  evaluators.add_evaluator(&pin_eval);

  int num_knotpoints = 30;
  double min_T = 3;
  double max_T = 3;
  auto mode = DirconMode<T>(evaluators, num_knotpoints, min_T, max_T);

  auto trajopt = Dircon<T>(&mode);

  const double R = 100;  // Cost on input effort
  auto u = trajopt.input();
  trajopt.AddRunningCost(u.transpose()*R*u);


  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto velocities_map = multibody::MakeNameToVelocitiesMap(plant);
  // // Print out position names
  // for (const auto& it : positions_map) {
  //   std::cout << it.first << std::endl;
  // }

  // trajopt.prog().SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Print file", "../snopt.out");
  trajopt.prog().SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 200);

  int nx = plant.num_positions() + plant.num_velocities();
  VectorXd times(num_knotpoints);
  MatrixXd states(nx, num_knotpoints);
  MatrixXd inputs(1, num_knotpoints);
  for (int i = 0; i < num_knotpoints; i++) {
    times(i) = min_T * i / (num_knotpoints - 1);
    states.col(i) = .1*Eigen::VectorXd::Random(nx);
    states.col(i).head(4) /= states.col(i).head(4).norm();
    inputs.col(i) = Eigen::VectorXd::Zero(1);
  }

  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(times, inputs);
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(times, states);

  trajopt.SetInitialTrajectory(traj_init_u, traj_init_x);

  auto x0 = trajopt.initial_state();
  // Set initial floating base orientation without overconstraining when
  // combined with quaternion norm constraint
  trajopt.prog().AddLinearConstraint(x0(positions_map.at("base_qx")) == .2);
  trajopt.prog().AddLinearConstraint(x0(positions_map.at("base_qy")) == .3);
  trajopt.prog().AddLinearConstraint(x0(positions_map.at("base_qz")) == -.2);
  trajopt.prog().AddLinearConstraint(x0(positions_map.at("base_qw")) >= .1);
  trajopt.prog().AddLinearConstraint(x0(plant.num_positions() + 
      velocities_map.at("base_wx")) == 0);
  trajopt.prog().AddLinearConstraint(x0(plant.num_positions() + 
      velocities_map.at("base_wy")) == 0);
  trajopt.prog().AddLinearConstraint(x0(plant.num_positions() + 
      velocities_map.at("base_wz")) == 0);


  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(trajopt.prog(), trajopt.prog().initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

  if (result.is_success()) {
    std::cout << "Success." << std::endl;
  } else {
    std::cout << "Failure." << std::endl;
  }

  // // Print out solution
  // VectorXd z = result.GetSolution(trajopt.decision_variables());
  // for (int i = 0; i < z.size(); i++) {
  //   std::cout << trajopt.decision_variables()(i) << " = " << z(i) << std::endl;
  // }


  // visualizer
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::ConnectTrajectoryVisualizer(&plant_vis, &builder, &scene_graph,
                                         pp_xtraj);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }

  return;
}
}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_autodiff) {
    dairlib::runDircon<drake::AutoDiffXd>();
  } else {
    dairlib::runDircon<double>();
  }
}
