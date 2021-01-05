#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

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
#include "examples/Spirit/animate_spirit.h"

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

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;

/// See runAnimate(
///    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
///    MultibodyPlant<double>* plant_double_ptr,
///    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
///    PiecewisePolynomial<double> pp_xtraj )
///
/// Takes the plants and scenegraph and a trajectory and
/// creates a visualization of that trajectory (example
/// built in the main file).
template<typename T>
void runAnimate(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double> *plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    PiecewisePolynomial<double> pp_xtraj
) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T> &plant = *plant_ptr;
  SceneGraph<double> &scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  // Print joint dictionary
//   for (auto const& element : positions_map)
//     cout << element.first << " = " << element.second << endl;
//   for (auto const& element : velocities_map)
//     cout << element.first << " = " << element.second << endl;

  multibody::connectTrajectoryVisualizer(plant_double_ptr,
                                         &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (true) {

    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(1);
  }
}

template void runAnimate(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr,
    drake::multibody::MultibodyPlant<double> *plant_double_ptr,
    std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr,
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj
); //NOLINT
}// namespace dairlib
