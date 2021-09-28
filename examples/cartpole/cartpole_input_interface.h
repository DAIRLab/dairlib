#pragma once

#include "epos/Definitions.h"
#include "epos/epos_util.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"

using std::string;

static constexpr int kNumActuators=1;
static constexpr double kMaxControllerDelay=0.1;

namespace dairlib {
class CartpoleInputInterface : public drake::systems::LeafSystem<double> {
 public:
  CartpoleInputInterface();
  void SetupEposDevice();


 private:

  drake::systems::EventStatus SendEposCommand(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* values) const;

  void CloseEposDevice();

  MAXON_HANDLE KeyHandle_= nullptr;
  int prev_timestamp_idx_;
  mutable bool error_flag_ = false;

};
}
