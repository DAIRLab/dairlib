#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"

namespace dairlib {
namespace multibody {


/// Creates and connects a trajectory playback visualizers using the given
/// builder and scene_graph.
/// Note: the plant cannot previously have been connectd to a visualizer
///       and the diagram must still be built
void connectTrajectoryVisualizer(
    const drake::multibody::MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    const drake::trajectories::Trajectory<double>& trajectory);

}  // namespace multibody
}  // namespace dairlib
