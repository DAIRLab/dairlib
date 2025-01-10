#pragma once
#include <vector>
#include <memory>
#include "knot_point.h"

namespace dairlib::systems::controllers::id_mpc {

class Timeline {
 public:
  const std::vector<double>& breaks() const {return breaks_;};
  void set_time_vector(const std::vector<double>& breaks);
  std::vector<KnotPointState> knot_states{};
  std::vector<std::unique_ptr<KnotPoint>> knots;

 private:
  std::vector<double> breaks_{};
};

}
