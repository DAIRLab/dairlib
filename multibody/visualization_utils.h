#pragma once

#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace dairlib {
namespace multibody {

/// Creates and connects a trajectory playback visualizers using the given
/// builder and scene_graph.
/// Note:
///  1. the plant cannot previously have been connected to a visualizer and the
///     diagram must still be built
///  2. pass in a empty plant, ball_plant, if you want to visualize the center
///     of mass position. The plant cannot be previously finalized.
void connectTrajectoryVisualizer(
    const drake::multibody::MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    const drake::trajectories::Trajectory<double>& trajectory,
    drake::multibody::MultibodyPlant<double>* ball_plant = nullptr);

}  // namespace multibody
}  // namespace dairlib
