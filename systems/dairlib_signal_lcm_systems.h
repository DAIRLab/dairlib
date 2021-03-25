#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_dairlib_signal.hpp"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Receives LCM message with type lcm_drake_signal, and outputs it as an
/// BasicVector as input.
class DairlibSignalReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit DairlibSignalReceiver(int signal_size);

 private:
  void UnpackLcmIntoVector(const drake::systems::Context<double>& context,
                           systems::TimestampedVector<double>* output) const;

  int signal_size_;
};

/// Receives BasicVector as input, and outputs it as an LCM message with type
/// lcm_drake_signal.
class DrakeSignalSender : public drake::systems::LeafSystem<double> {
 public:
  explicit DrakeSignalSender(const std::vector<std::string>& signal_names,
                             double stride_period);

 private:
  void PackVectorIntoLcm(const drake::systems::Context<double>& context,
                         dairlib::lcmt_dairlib_signal* output) const;

  std::vector<std::string> signal_names_;
  int signal_size_;

  // For testing
  double stride_period_;
};

}  // namespace systems
}  // namespace dairlib
