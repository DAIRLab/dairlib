#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "attic/multibody/rigidbody_utils.h"
#include "multibody/multibody_utils.h"

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_cassie_pelvis_out.hpp"
#include "dairlib/lcmt_vectornav_out.hpp"
#include "dairlib/lcmt_cassie_leg_out.hpp"
#include "dairlib/lcmt_elmo_out.hpp"
#include "dairlib/lcmt_cassie_joint_out.hpp"

namespace dairlib {
namespace systems {

/// SimCassieSensorAggregator is a block that gathers controller input,
/// gyroscope, accelerometer and plant state, and packs them into
/// dairlib::lcmt_cassie_out.
class SimCassieSensorAggregator : public drake::systems::LeafSystem<double> {
 public:
  explicit SimCassieSensorAggregator(const RigidBodyTree<double>& tree);

  const drake::systems::InputPort<double>& get_input_port_input() const {
    return this->get_input_port(input_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_acce() const {
    return this->get_input_port(acce_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_gyro() const {
    return this->get_input_port(gyro_input_port_);
  }

 private:
  void Aggregate(const drake::systems::Context<double>& context,
                 dairlib::lcmt_cassie_out* cassie_out_msg) const;

  int input_input_port_;
  int state_input_port_;
  int acce_input_port_;
  int gyro_input_port_;

  int num_positions_;
  int num_velocities_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  std::map<std::string, int> actuatorIndexMap_;
};

}  // namespace systems
}  // namespace dairlib
