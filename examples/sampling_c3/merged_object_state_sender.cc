#include "examples/sampling_c3/merged_object_state_sender.h"

#include "drake/common/value.h"

namespace dairlib {
namespace systems {

using drake::AbstractValue;
using drake::systems::Context;

MergedObjectStateSender::MergedObjectStateSender(
    const std::vector<std::string>& object_names,
    const std::vector<std::string>& merged_object_names,
    const std::vector<std::vector<std::string>>& object_groups_for_merging)
    : object_names_(object_names),
      merged_object_names_(merged_object_names),
      object_groups_for_merging_(object_groups_for_merging) {
  this->set_name("MergedObjectStateSender");

  // Create input ports for each object state
  for (size_t i = 0; i < object_names.size(); ++i) {
    const std::string& name = object_names[i];
    auto port_index = this->DeclareAbstractInputPort(
                              "object_state_" + name,
                              drake::Value<dairlib::lcmt_object_state>{})
                          .get_index();
    object_state_input_ports_[name] = port_index;
    object_state_input_port_indices_.push_back(port_index);
  }

  // Create single output port for all merged states (vector)
  merged_states_output_port_ =
      this->DeclareAbstractOutputPort(
              "merged_states",
              // Allocator
              []() {
                return AbstractValue::Make(
                    std::vector<dairlib::lcmt_object_state>{});
              },
              // Calculator
              [this](const Context<double>& context, AbstractValue* output) {
                auto& output_vec =
                    output->get_mutable_value<std::vector<lcmt_object_state>>();
                this->CalcMergedOutputs(context, &output_vec);
              })
          .get_index();

  // Create individual output ports for each merged object
  for (size_t i = 0; i < merged_object_names.size(); ++i) {
    const std::string& merged_name = merged_object_names[i];
    auto port_index =
        this->DeclareAbstractOutputPort(
                "merged_state_" + merged_name,
                // Allocator
                []() {
                  return AbstractValue::Make(dairlib::lcmt_object_state{});
                },
                // Calculator - captures index by value
                [this, i](const Context<double>& context,
                          AbstractValue* output) {
                  auto& out_msg =
                      output->get_mutable_value<lcmt_object_state>();
                  this->CalcSingleMergedOutput(context, i, &out_msg);
                })
            .get_index();
    merged_state_output_ports_[merged_name] = port_index;
    merged_state_output_port_indices_.push_back(port_index);
  }
}

void MergedObjectStateSender::CalcMergedOutputs(
    const Context<double>& context,
    std::vector<dairlib::lcmt_object_state>* output) const {
  // Clear and resize output vector
  output->clear();
  output->resize(merged_object_names_.size());

  // Process each group
  for (size_t group_index = 0; group_index < object_groups_for_merging_.size();
       ++group_index) {
    const auto& group = object_groups_for_merging_[group_index];
    const std::string& merged_name = merged_object_names_[group_index];

    auto& merged_output = (*output)[group_index];

    // Initialize the output message
    merged_output.utime = static_cast<int64_t>(context.get_time() * 1e6);
    merged_output.object_name = merged_name;

    // Clear and prepare to accumulate positions (no velocities)
    merged_output.position_names.clear();
    merged_output.position.clear();
    merged_output.velocity_names.clear();
    merged_output.velocity.clear();

    // Merge states from all objects in this group
    for (const auto& obj_name : group) {
      const auto& input_port_index = object_state_input_ports_.at(obj_name);
      const auto& obj_state =
          this->get_input_port(input_port_index)
              .template Eval<dairlib::lcmt_object_state>(context);

      // Append positions with prefixed names
      for (int i = 0; i < obj_state.num_positions; ++i) {
        merged_output.position_names.push_back(obj_name + "_" +
                                               obj_state.position_names[i]);
        merged_output.position.push_back(obj_state.position[i]);
      }
    }

    // Update counts (no velocities)
    merged_output.num_positions =
        static_cast<int32_t>(merged_output.position.size());
    merged_output.num_velocities = 0;
  }
}

void MergedObjectStateSender::CalcSingleMergedOutput(
    const Context<double>& context, size_t group_index,
    dairlib::lcmt_object_state* output) const {
  const auto& group = object_groups_for_merging_[group_index];
  const std::string& merged_name = merged_object_names_[group_index];

  // Initialize the output message
  output->utime = static_cast<int64_t>(context.get_time() * 1e6);
  output->object_name = merged_name;

  // Clear and prepare to accumulate positions (no velocities)
  output->position_names.clear();
  output->position.clear();
  output->velocity_names.clear();
  output->velocity.clear();

  // Merge states from all objects in this group
  for (const auto& obj_name : group) {
    const auto& input_port_index = object_state_input_ports_.at(obj_name);
    const auto& obj_state =
        this->get_input_port(input_port_index)
            .template Eval<dairlib::lcmt_object_state>(context);

    // Append positions with prefixed names
    for (int i = 0; i < obj_state.num_positions; ++i) {
      output->position_names.push_back(obj_name + "_" +
                                       obj_state.position_names[i]);
      output->position.push_back(obj_state.position[i]);
    }
  }

  // Update counts (no velocities)
  output->num_positions = static_cast<int32_t>(output->position.size());
  output->num_velocities = 0;
}

}  // namespace systems
}  // namespace dairlib
