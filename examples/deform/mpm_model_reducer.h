#pragma once

#include <string>
#include <vector>

#include "dairlib/lcmt_elastoplastic_network.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Converts MPM points to LCM type lcmt_elastoplastic_network.
class MpmPointsToReducedModel : public drake::systems::LeafSystem<double> {
 public:
  MpmPointsToReducedModel(Eigen::Matrix3Xd support_directions,
                          Eigen::Matrix2Xi connections);

  // Input port
  const drake::systems::InputPort<double>& get_input_port_lcmt_material_points()
      const {
    return this->get_input_port(lcmt_material_points_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>&
  get_output_port_lcmt_elastoplastic_network() const {
    return this->get_output_port(lcmt_elastoplastic_network_output_port_);
  }

 private:
  void OutputReducedModelNetworkLcm(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_elastoplastic_network* output) const;

  const int n_support_directions_;
  const Eigen::Matrix3Xd support_directions_;
  const int n_connections_;
  const Eigen::Matrix2Xi connections_;
  std::vector<std::vector<int>> connections_data_;

  drake::systems::InputPortIndex lcmt_material_points_input_port_;
  drake::systems::OutputPortIndex lcmt_elastoplastic_network_output_port_;
};

}  // namespace systems
}  // namespace dairlib
