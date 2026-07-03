#pragma once

#include <memory>
#include <string>
#include <vector>

#include <drake/geometry/meshcat.h>

#include "multibody/multipose_visualizer.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace systems {

/// Draws the nodes and connections of a lcmt_elastoplastic_network through
/// meshcat: the nodes via a MultiposeVisualizer, and the connections as
/// cylinder segments.
class ElastoPlasticModelInterpreter
    : public drake::systems::LeafSystem<double> {
 public:
  ElastoPlasticModelInterpreter(
      const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
      int n_network_points, const std::string& node_model_file,
      Eigen::VectorXd color);

  // Input port
  const drake::systems::InputPort<double>&
  get_input_port_lcmt_elastoplastic_network() const {
    return this->get_input_port(lcmt_elastoplastic_network_input_port_);
  }

 private:
  // Discrete update event to draw the nodes and connections
  drake::systems::EventStatus DrawNetwork(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  int n_network_points_;

  drake::systems::InputPortIndex lcmt_elastoplastic_network_input_port_;

  drake::systems::DiscreteStateIndex last_update_time_index_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  const drake::geometry::Cylinder cylinder_ =
      drake::geometry::Cylinder(0.002, 1.0);
  const std::string connection_path_ = "elastoplastic_connections";
  drake::geometry::Rgba connection_color_;

  // Draws the network node points. The node model's "base_link" is welded to
  // world since each node is represented as a translation-only (x/y/z
  // prismatic) point relative to it, not a floating body.
  std::unique_ptr<multibody::MultiposeVisualizer> node_visualizer_;
};

}  // namespace systems
}  // namespace dairlib
