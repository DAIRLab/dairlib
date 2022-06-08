#pragma once

#include <dairlib/lcmt_swing_foot_spline_params.hpp>
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class SwingFootParamsSender : public drake::systems::LeafSystem<double> {
 public:
  SwingFootParamsSender(int n_knot);

 protected:
  void CalcOutput(const drake::systems::Context<double>& context,
                       dairlib::lcmt_swing_foot_spline_params* output) const;

 private:
  const int n_knot_;
};

}
}


