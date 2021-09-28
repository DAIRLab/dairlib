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
class CartpoleOutputInterface : public drake::systems::LeafSystem<double> {
 public:
  CartpoleOutputInterface();

 private:


  MAXON_HANDLE KeyHandle_= nullptr;
};
}