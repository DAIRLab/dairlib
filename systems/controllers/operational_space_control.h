#pragma once

#include <vector>
#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {

// Time-based Two-state Finite State Machine
class OperationalSpaceControl : public drake::systems::LeafSystem<double> {
 public:
  OperationalSpaceControl(std::vector<OscTrackingData> tracking_data_vec);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }


  std::vector<OscTrackingData> GetTrackingDataVector() {
    return tracking_data_vec_;
  }

 private:
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        dairlab::systems::TimestampedVector<double>* control) const;

  int state_port_;

  std::vector<OscTrackingData> tracking_data_vec_;
};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
