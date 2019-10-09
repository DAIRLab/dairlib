#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "drake/common/find_resource.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"

/// Simple example using DirconDistanceData. Starting from an acrobot, adds
/// a distance constraint between the base and a point on the lower link. This
/// converts the system to a simple passive pendulum, as the lower and upper
/// links will be fixed relative to one another.
/// Runs DIRCON from a given initial condition.
namespace dairlib {
namespace {
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Parser;
using Eigen::Vector3d;
using drake::trajectories::PiecewisePolynomial;
using std::vector;
using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;


// Fixed path to double pendulum SDF model.
static const char* const kDoublePendulumSdfPath =
  "drake/examples/acrobot/Acrobot.urdf";

void runDircon() {
  const std::string sdf_path =
      drake::FindResourceOrThrow(kDoublePendulumSdfPath);
  MultibodyPlant<double> plant;

  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  Parser parser(&plant, &scene_graph);

  parser.AddModelFromFile(sdf_path);

  plant.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());

  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("base_link"),
      drake::math::RigidTransform<double>());

  plant.Finalize();

  const Body<double>& base = plant.GetBodyByName("base_link");
  const Body<double>& lower_link = plant.GetBodyByName("lower_link");

  Vector3d pt1;
  pt1 << 0, 0, 0;
  Vector3d pt2;
  pt2 << -1, 0, 0;
  double distance = .7;

  auto distanceConstraint = DirconDistanceData<double>(plant, base, pt1,
      lower_link, pt2, distance);

  std::vector<DirconKinematicData<double>*> constraints;
  constraints.push_back(&distanceConstraint);
  auto constraintSet = DirconKinematicDataSet<double>(plant, &constraints);

  auto options = DirconOptions(1);

  std::vector<int> timesteps;
  timesteps.push_back(30);
  std::vector<double> min_dt;
  min_dt.push_back(.1);
  std::vector<double> max_dt;
  max_dt.push_back(.1);

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&constraintSet);

  std::vector<DirconOptions> options_list;
  options_list.push_back(options);

  auto trajopt = std::make_shared<HybridDircon<double>>(plant,
      timesteps, min_dt, max_dt, dataset_list, options_list);

  const double R = 10;  // Cost on input effort
  auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R*u);

  auto x0 = trajopt->initial_state();
  trajopt->AddLinearConstraint(x0(0) == 1);

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

  if (result.is_success()) {
    std::cout << "Success." << std::endl;
  } else {
    std::cout << "Failure." << std::endl;
  }

  // visualizer
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt->ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                         pp_xtraj);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.5);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }

  return;
}
}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::runDircon();
}
