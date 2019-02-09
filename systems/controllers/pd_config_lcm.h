#pragma once

#include <string>
#include <map>

#include "dairlib/lcmt_pd_config.hpp"
#include "drake/systems/framework/leaf_system.h"
#include "attic/multibody/rigidbody_utils.h"
#include "systems/controllers/linear_controller.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie PD configuration channel with LCM type lcmt_cassie_pd_config,
/// and outputs the CassiePDConfig as Context
class PDConfigReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit PDConfigReceiver(const RigidBodyTree<double>& tree);

 private:
  void CopyConfig(const drake::systems::Context<double>& context,
                  LinearConfig* output) const;

  const RigidBodyTree<double>* tree_;
  std::map<string, int> actuatorIndexMap_;
  std::map<int, int> actuatorToPositionIndexMap_;
  std::map<int, int> actuatorToVelocityIndexMap_;
};

}  // namespace systems
}  // namespace dairlib
