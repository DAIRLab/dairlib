#pragma once
#include <vector>
#include "knot_point_state.h"

namespace dairlib::systems::controllers::id_mpc {

class Timeline {
 public:

  template<typename T>
  void Update(const drake::VectorX<T> stacked_decision_vars);

  int nknots() const { return breaks.size(); }

  int total_vars() const {
    return knots.front().get_dynamics().variable_count() * nknots();
  }
  int total_dynamics_constraints() const {
    return knots.front().get_dynamics().nx() * (nknots() - 1);
  };

  std::vector<double> breaks{};
  std::vector<KnotPointState> knots{};
};

}
