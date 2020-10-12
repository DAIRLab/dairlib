#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Receives LCM message with type lcm_drake_signal, and outputs it as an
/// BasicVector as input.
class DrakeSignalReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit DrakeSignalReceiver(int signal_size);

 private:
  void UnpackLcmIntoVector(const drake::systems::Context<double>& context,
                           drake::systems::BasicVector<double>* output) const;

  int signal_size_;
};

/// Receives BasicVector as input, and outputs it as an LCM message with type
/// lcm_drake_signal.
class DrakeSignalSender : public drake::systems::LeafSystem<double> {
 public:
  explicit DrakeSignalSender(const std::vector<std::string>& signal_names);

 private:
  void PackVectorIntoLcm(const drake::systems::Context<double>& context,
                         drake::lcmt_drake_signal* output) const;

  std::vector<std::string> signal_names_;
  int signal_size_;
};

}  // namespace systems
}  // namespace dairlib
