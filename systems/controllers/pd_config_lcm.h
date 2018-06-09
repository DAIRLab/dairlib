#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "multibody/rbt_utils.h"
#include "systems/controllers/linear_controller.h"
#include "dairlib/lcmt_pd_config.hpp"

namespace dairlib {
namespace systems {

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie PD configuration channel with LCM type lcmt_cassie_pd_config, 
/// and outputs the CassiePDConfig as Context
class PDConfigReceiver : public drake::systems::LeafSystem<double> {
 public:
  PDConfigReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyConfig(const drake::systems::Context<double>& context,
                  LinearConfig* output) const;

  const RigidBodyTree<double>* tree_;
  std::map<string, int> actuatorIndexMap_;
  std::map<int, int> actuatorToPositionIndexMap_;
  std::map<int, int> actuatorToVelocityIndexMap_;
};
}
}