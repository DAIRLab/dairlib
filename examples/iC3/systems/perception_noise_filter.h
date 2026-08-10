#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>

#include "common/find_resource.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/state_vector.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using drake::systems::BasicVector;

using drake::systems::InputPort;
using drake::systems::OutputPort; 
using drake::systems::InputPortIndex;
using drake::systems::OutputPortIndex; 
/*
  When not tracking iC3, commands osc to hold nominal position
  We solve c3 with plate initially at origin, so also translates positions back into world frame
*/

namespace dairlib {

class PerceptionNoiseFilter : public drake::systems::LeafSystem<double> {
 public:

  explicit PerceptionNoiseFilter(bool add_noise);
  
  const InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_input_port_);
  }

  const OutputPort<double>& get_output_port_object_state() const {
    return this->get_output_port(object_state_output_port_);
  }


 private:
   void OutputNoisyObjectState(
      const drake::systems::Context<double>& context,
      systems::StateVector<double>* output_state) const;

  InputPortIndex object_state_input_port_;
  OutputPortIndex object_state_output_port_;

  bool add_noise_;

};

} // namespace dairlib