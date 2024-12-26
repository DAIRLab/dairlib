#pragma once
#include <vector>
#include "knot_point_state.h"

namespace dairlib::systems::controllers::id_mpc {

struct Timeline {
  std::vector<double> breaks{};
  std::vector<KnotPointState> knots{};
};

}
