#pragma once

#include "dairlib/lcmt_footstep_target.hpp"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {
class FootstepSender : public drake::systems::LeafSystem<double> {
 public:
  FootstepSender();
 private:
  void CopyMessage(const drake::systems::Context<double>& context,
                   dairlib::lcmt_footstep_target* msg) const;
};

class FootstepReceiver : public drake::systems::LeafSystem<double> {
 public:
  FootstepReceiver();
 private:
  void CopyMessage(const drake::systems::Context<double>& context,
                   drake::systems::BasicVector<double>* y) const;
};

}

