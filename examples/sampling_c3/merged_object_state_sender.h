#pragma once

#include <map>
#include <string>
#include <vector>

#include <dairlib/lcmt_object_state.hpp>

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// MergedObjectStateSender is a LeafSystem that takes multiple object states
/// as inputs and merges them according to a grouping configuration.
///
/// For example, if you have objects ["obj_A", "obj_B", "obj_C"] and you want
/// to merge them into groups [["obj_A", "obj_B"], ["obj_C"]], this system will:
/// - Have 3 input ports (one for each object state)
/// - Have 2 output ports (one for each merged group)
///
/// The merged state combines positions from all objects in a group into a
/// single lcmt_object_state message. Velocities are not included in the output.
class MergedObjectStateSender : public drake::systems::LeafSystem<double> {
 public:
  /// Constructor
  /// @param object_names List of all object names that will have input ports
  /// @param merged_object_names Names for the merged output objects
  /// @param object_groups_for_merging Groups of object names to merge together
  MergedObjectStateSender(
      const std::vector<std::string>& object_names,
      const std::vector<std::string>& merged_object_names,
      const std::vector<std::vector<std::string>>& object_groups_for_merging);

  /// Get the input port for a specific object by name
  const drake::systems::InputPort<double>& get_input_port_object_state(
      const std::string& object_name) const {
    return this->get_input_port(object_state_input_ports_.at(object_name));
  }

  /// Get the input port for a specific object by index
  const drake::systems::InputPort<double>& get_input_port_object_state(
      int index) const {
    return this->get_input_port(object_state_input_port_indices_.at(index));
  }

  /// Get the output port for a specific merged object by name
  const drake::systems::OutputPort<double>& get_output_port_merged_state(
      const std::string& merged_object_name) const {
    return this->get_output_port(
        merged_state_output_ports_.at(merged_object_name));
  }

  /// Get the output port for a specific merged object by index
  const drake::systems::OutputPort<double>& get_output_port_merged_state(
      int index) const {
    return this->get_output_port(merged_state_output_port_indices_.at(index));
  }

  /// Get the number of object input ports
  int num_object_inputs() const {
    return static_cast<int>(object_names_.size());
  }

  /// Get the number of merged output ports
  int num_merged_outputs() const {
    return static_cast<int>(merged_object_names_.size());
  }

 private:
  /// Output calculation method for a single merged state at given index
  void CalcSingleMergedOutput(const drake::systems::Context<double>& context,
                              size_t group_index,
                              dairlib::lcmt_object_state* output) const;

  // Configuration
  std::vector<std::string> object_names_;
  std::vector<std::string> merged_object_names_;
  std::vector<std::vector<std::string>> object_groups_for_merging_;

  // Input ports - one per object
  std::map<std::string, drake::systems::InputPortIndex>
      object_state_input_ports_;
  std::vector<drake::systems::InputPortIndex> object_state_input_port_indices_;

  // Individual output ports - one per merged object
  std::map<std::string, drake::systems::OutputPortIndex>
      merged_state_output_ports_;
  std::vector<drake::systems::OutputPortIndex>
      merged_state_output_port_indices_;
};

}  // namespace systems
}  // namespace dairlib
