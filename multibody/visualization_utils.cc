#include "multibody/visualization_utils.h"

#include "drake/systems/primitives/trajectory_source.h"
#include "systems/primitives/subvector_pass_through.h"

namespace dairlib {
namespace multibody {
using drake::geometry::DrakeVisualizer;
using drake::multibody::MultibodyPlant;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::trajectories::Trajectory;
using systems::SubvectorPassThrough;

void connectTrajectoryVisualizer(const MultibodyPlant<double>* plant,
    drake::systems::DiagramBuilder<double>* builder,
    drake::geometry::SceneGraph<double>* scene_graph,
    const Trajectory<double>& trajectory) {
  auto traj_source = builder->AddSystem<drake::systems::TrajectorySource>(
        trajectory);
  auto passthrough = builder->AddSystem<SubvectorPassThrough>(
    plant->num_positions() + plant->num_velocities(), 0,
    plant->num_positions());
  builder->Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose =
      builder->AddSystem<MultibodyPositionToGeometryPose<double>>(*plant);
  builder->Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder->Connect(to_pose->get_output_port(),
      scene_graph->get_source_pose_port(plant->get_source_id().value()));

  DrakeVisualizer<double>::AddToBuilder(builder, *scene_graph);
}

}  // namespace multibody
}  // namespace dairlib
