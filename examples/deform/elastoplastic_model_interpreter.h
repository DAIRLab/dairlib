#pragma once

#include <string>
#include <vector>

#include <drake/geometry/meshcat.h>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Converts lcmt_elastoplastic_network to types that can be visualized:  a
/// lcmt_timestamped_saved_traj for the points, and line segments for the
/// connections.
class ElastoPlasticModelInterpreter
    : public drake::systems::LeafSystem<double> {
 public:
  ElastoPlasticModelInterpreter(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      int n_network_points);

  // Input port
  const drake::systems::InputPort<double>&
  get_input_port_lcmt_elastoplastic_network() const {
    return this->get_input_port(lcmt_elastoplastic_network_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>&
  get_output_port_points_lcmt_timestamped_saved_traj() const {
    return this->get_output_port(
        points_lcmt_timestamped_saved_traj_output_port_);
  }

 private:
  // Output function
  void OutputReducedModelPointsLcm(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output) const;

  // Discrete update event to draw connections
  drake::systems::EventStatus DrawConnections(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int n_network_points_;

  drake::systems::InputPortIndex lcmt_elastoplastic_network_input_port_;
  drake::systems::OutputPortIndex
      points_lcmt_timestamped_saved_traj_output_port_;

  drake::systems::DiscreteStateIndex last_update_time_index_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const drake::geometry::Cylinder cylinder_ =
      drake::geometry::Cylinder(0.002, 1.0);
  const std::string connection_path_ = "elastoplastic_connections";
  drake::geometry::Rgba connection_color_ =
      drake::geometry::Rgba(0.949, 0.757, 0.0, 1.0);
};

}  // namespace systems
}  // namespace dairlib
