#pragma once

namespace dairlib {

  /// See runAnimate(
  ///    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
  ///    MultibodyPlant<double>* plant_double_ptr,
  ///    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
  ///    PiecewisePolynomial<double> pp_xtraj )
  ///
  /// Takes the plants and scenegraph and a trajectory and 
  /// creates a visualization of that trajectory (example
  /// built in the main file).
void runAnimate(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr,
    drake::multibody::MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr,
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj
);

void printFoo();

}

