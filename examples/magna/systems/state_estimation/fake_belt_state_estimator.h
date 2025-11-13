#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

class FakeBeltStateEstimator : public drake::systems::LeafSystem<double> {
 public:
  explicit FakeBeltStateEstimator(int num_keypoints);
  const drake::systems::InputPort<double>& get_input_port_franka_state() const {
    return this->get_input_port(franka_state_input_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_keypoint_state()
      const {
    return this->get_output_port(keypoint_state_output_port_);
  }

 private:
  drake::systems::InputPortIndex franka_state_input_port_;
  drake::systems::OutputPortIndex keypoint_state_output_port_;
};

}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
