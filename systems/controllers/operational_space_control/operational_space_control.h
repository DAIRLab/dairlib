#pragma once

#include <vector>
#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {


// The difference between
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



  // RBT's. OSC calculates feedback position/velocity from tree with springs,
  // but in the optimization it uses tree without springs. The reason of using
  // the one without spring is that the OSC cannot track desired acceleration
  // instantaneously when springs exist. (relative degrees of 4)
  // The springs here refer to the compliant components in the robots.
  RigidBodyTree<double>* tree_with_springs_;
  RigidBodyTree<double>* tree_without_springs_;
  //TODO: You'll send tree's in the function of CheckOscTrackingData to
  //calculate posistion, etc.:

  std::vector<OscTrackingData> tracking_data_vec_;
};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
