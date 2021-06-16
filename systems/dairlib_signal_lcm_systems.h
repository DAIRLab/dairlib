#pragma once

#include <map>
#include <string>
#include <vector>

#include "dairlib/lcmt_dairlib_signal.hpp"
#include "dairlib/lcmt_timestamped_vector.hpp"
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
class DairlibSignalSender : public drake::systems::LeafSystem<double> {
 public:
  explicit DairlibSignalSender(const std::vector<std::string>& signal_names,
                               double stride_period);

  explicit DairlibSignalSender(const std::vector<std::string>& signal_names);

 private:
  void PackVectorIntoLcm(const drake::systems::Context<double>& context,
                         dairlib::lcmt_dairlib_signal* output) const;

  std::vector<std::string> signal_names_;
  int signal_size_;

  // For testing
  bool with_hacks_;
  double stride_period_;
};

/*class TimestampedVectorReceiver : public drake::systems::LeafSystem<double> {
 public:
  explicit TimestampedVectorReceiver(int signal_size);

 private:
  void UnpackLcmIntoVector(const drake::systems::Context<double>& context,
                           systems::TimestampedVector<double>* output) const;

  int signal_size_;
};*/

class TimestampedVectorSender : public drake::systems::LeafSystem<double> {
 public:
  explicit TimestampedVectorSender(int signal_size);

 private:
  void PackVectorIntoLcm(const drake::systems::Context<double>& context,
                         dairlib::lcmt_timestamped_vector* output) const;

  int signal_size_;
};

}  // namespace systems
}  // namespace dairlib
