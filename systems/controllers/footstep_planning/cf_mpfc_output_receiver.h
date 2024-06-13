#pragma once
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {

class CFMPFCOutputReceiver : public drake::systems::LeafSystem<double> {
 public:
  CFMPFCOutputReceiver();

};

}
