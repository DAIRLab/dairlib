#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "multibody/rbt_utils.h"
#include "systems/controllers/linear_controller.h"
#include "dairlib/lcmt_pd_config.hpp"

namespace dairlib {
namespace systems {

using dairlib::systems::TimestampedVector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

/// Receives the output of an LcmSubsriberSystem that subsribes to the
/// Cassie PD configuration channel with LCM type lcmt_cassie_pd_config, 
/// and outputs the CassiePDConfig as Context
class PDConfigReceiver : public systems::LeafSystem<double> {
 public:
  PDConfigReceiver(RigidBodyTree<double>& tree);


 private:
  void CopyConfig(const systems::Context<double>& context,
                  LinearConfig* output) const;

  const RigidBodyTree<double>* tree_;
  map<string, int> positionIndexMap_;
  map<string, int> velocityIndexMap_;
  map<string, int> actuatorIndexMap_;
};
}
}