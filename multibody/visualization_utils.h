#pragma once

#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/drake_visualizer.h"


namespace dairlib {
namespace multibody {

/// Construct a ball plant for visualization
std::unique_ptr<drake::multibody::MultibodyPlant<double>> ConstructBallPlant(
    drake::geometry::SceneGraph<double>* scene_graph);

/// Creates and connects a trajectory playback visualizers using the given
/// builder and scene_graph.
/// Note:
///  1. the plant cannot previously have been connected to a visualizer and the
///     diagram must still be built
///  2. if you want to visualize the center of mass position, provide ball_plant
///     as an input. The ball can be constructed by the function
///     ConstructBallPlant().
void ConnectTrajectoryVisualizer(
    const drake::multibody::MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    const drake::trajectories::Trajectory<double>& trajectory);
void ConnectTrajectoryVisualizer(
    const drake::multibody::MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    const drake::trajectories::Trajectory<double>& trajectory,
    const drake::multibody::MultibodyPlant<double>& ball_plant);

}  // namespace multibody
}  // namespace dairlib
