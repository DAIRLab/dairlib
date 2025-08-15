#pragma once

#include <drake/geometry/shape_specification.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/discrete_values.h>

#include "dairlib/lcmt_object_state.hpp"
#include "dairlib/lcmt_densetact_measurement_data.hpp"

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

class LcmDensetactDrawer : public drake::systems::LeafSystem<double> {
 public:
  explicit LcmDensetactDrawer(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat);

 private:
  drake::systems::EventStatus DrawDensetact(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  drake::systems::InputPortIndex densetact_input_port_;

  drake::systems::DiscreteStateIndex last_update_time_index_;

  const drake::geometry::Sphere sphere_for_densetact_ =
      drake::geometry::Sphere(0.016);

  const std::vector<std::string> densetact_target_paths_ = {"visualizer/trifingeredu/densetact_0/densetact_data", "visualizer/trifingeredu/densetact_120/densetact_data"};
};

}  // namespace dairlib::systems