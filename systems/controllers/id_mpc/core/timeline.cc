#include "timeline.h"
#include <iostream>
namespace dairlib::systems::controllers::id_mpc {

void Timeline::set_time_vector(const std::vector<double> &breaks) {
  if (breaks.size() != knot_states.size()) {
    std::cerr << "Can't assign breaks of size " << breaks.size() <<
                 " to MPC timeline of size " << knot_states.size() << std::endl;
  }
  DRAKE_DEMAND(breaks.size() == knot_states.size());
  breaks_ = breaks;

  for (int i = 0; i < breaks_.size(); ++i) {
    DRAKE_ASSERT(i == 0 || breaks_.at(i-1) < breaks_.at(i));
    knot_states.at(i).UpdateTimestamp(breaks_.at(i));
  }

}

}