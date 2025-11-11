#pragma once

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace visualization {

class C3BeltTargetStateDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit C3BeltTargetStateDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      int num_keypoints, const int end_effector_state_size = 6,
      const int keypoint_state_size = 3,
      const std::string& c3_state_path = "visualization/c3_state");

  const drake::systems::InputPort<double>& get_input_port_c3_state_target()
      const {
    return this->get_input_port(c3_state_target_input_port_);
  }

 private:
  drake::systems::EventStatus DrawC3State(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  drake::systems::InputPortIndex c3_state_target_input_port_;

  int num_keypoints_;
  int end_effector_state_size_;
  int keypoint_state_size_;

  drake::systems::DiscreteStateIndex last_update_time_index_;

  const drake::geometry::Cylinder cylinder_for_ee_ =
      drake::geometry::Cylinder(0.0025, 0.05);
  const drake::geometry::Sphere sphere_for_keypoint_ =
      drake::geometry::Sphere(0.01);
  const std::string c3_state_path_ = "c3_belt_state";
};
}  // namespace visualization
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
