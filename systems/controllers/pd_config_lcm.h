#pragma once

#include <string>
#include <map>

#include "dairlib/lcmt_pd_config.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "attic/multibody/rigidbody_utils.h"
#include "systems/controllers/linear_controller.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace systems {

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie PD configuration channel with LCM type lcmt_cassie_pd_config,
/// and outputs the CassiePDConfig as Context
class PDConfigReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit PDConfigReceiver(const RigidBodyTree<double>& tree);

  explicit PDConfigReceiver(
    const drake::multibody::MultibodyPlant<double>& plant);

 private:
  void CopyConfig(const drake::systems::Context<double>& context,
                  LinearConfig* output) const;

  std::map<string, int> actuatorIndexMap_;
  std::map<int, int> actuatorToPositionIndexMap_;
  std::map<int, int> actuatorToVelocityIndexMap_;
  int num_positions_;
  int num_velocities_;
  int num_actuators_;
};

}  // namespace systems
}  // namespace dairlib
