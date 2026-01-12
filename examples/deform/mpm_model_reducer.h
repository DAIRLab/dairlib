#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Converts sample costs and configurations to LCM type
/// lcmt_timestamped_saved_traj
class MpmPointsToReducedModelPoints
    : public drake::systems::LeafSystem<double> {
 public:
  MpmPointsToReducedModelPoints(Eigen::Matrix3Xd support_directions);

  // Input port
  const drake::systems::InputPort<double>& get_input_port_lcmt_material_points()
      const {
    return this->get_input_port(lcmt_material_points_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>&
  get_output_port_lcmt_timestamped_saved_traj() const {
    return this->get_output_port(lcmt_timestamped_saved_traj_output_port_);
  }

 private:
  void OutputReducedModelPointsLcm(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;

  int n_support_directions_;
  Eigen::Matrix3Xd support_directions_;

  drake::systems::InputPortIndex lcmt_material_points_input_port_;
  drake::systems::OutputPortIndex lcmt_timestamped_saved_traj_output_port_;
};

}  // namespace systems
}  // namespace dairlib
