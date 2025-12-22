#pragma once

#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace visualization {

class C3BeltStateDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit C3BeltStateDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      int num_keypoints, bool is_target_state = false,
      const int end_effector_state_size = 6, const int keypoint_state_size = 3,
      const std::string& c3_state_path = "c3_state",
      double spring_stiffness = 100.0, double spring_rest_length = 0.27);

  const drake::systems::InputPort<double>& get_input_port_c3_state() const {
    return this->get_input_port(c3_state_input_port_);
  }

 private:
  drake::systems::EventStatus DrawC3State(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  drake::systems::InputPortIndex c3_state_input_port_;

  int num_keypoints_;
  bool is_target_state_;
  int end_effector_state_size_;
  int keypoint_state_size_;
  double spring_stiffness_;
  double spring_rest_length_;

  drake::systems::DiscreteStateIndex last_update_time_index_;

  const drake::geometry::Box box_for_ee_ =
      drake::geometry::Box(0.01, 0.015, 0.01);
  const drake::geometry::Cylinder cylinder_for_ee_ =
      drake::geometry::Cylinder(0.0025, 0.05);
  const drake::geometry::Sphere sphere_for_keypoint_ =
      drake::geometry::Sphere(0.01);
  const double spring_arrow_radius_ = 0.0025;
  const drake::geometry::Cylinder spring_arrow_cylinder_ =
      drake::geometry::Cylinder(spring_arrow_radius_, 1.0);
  const drake::geometry::MeshcatCone spring_arrowhead_ =
      drake::geometry::MeshcatCone(spring_arrow_radius_ * 2.0,
                                   spring_arrow_radius_ * 2.0,
                                   spring_arrow_radius_ * 2.0);
  const std::string c3_state_path_ = "c3_belt_state";

  // Helper struct for color sets
  struct ColorSet {
    drake::geometry::Rgba ee_color;
    drake::geometry::Rgba keypoint_color;
  };

  // Get color set based on whether this is target state
  static ColorSet GetColorSet(bool is_target_state);
};
}  // namespace visualization
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
