#pragma once

#include <drake/geometry/shape_specification.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/discrete_values.h>

#include "dairlib/lcmt_object_state.hpp"

#include "drake/geometry/meshcat.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {
class LcmCubeTargetDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmCubeTargetDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat);

  const drake::systems::InputPort<double>& get_input_port_cube_target() const {
    return this->get_input_port(cube_target_input_port_);
  }

 private:
  drake::systems::EventStatus DrawCubeTarget(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  drake::systems::InputPortIndex cube_target_input_port_;

  drake::systems::DiscreteStateIndex last_update_time_index_;

  const drake::geometry::Box box_for_cube_target_ =
      drake::geometry::Box(0.065, 0.065, 0.065);
  const std::string cube_target_path_ = "cube_target/";
};
}  // namespace dairlib::systems