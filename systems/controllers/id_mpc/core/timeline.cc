#include "timeline.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::VectorX;

template<typename T>
void Timeline::Update(const VectorX<T> stacked_decision_vars) {
  int n = breaks.size() - 1;
  int nvars = knots.front().get_dynamics().variable_count();
  int nx = knots.front().get_dynamics().nx();
  int nv = knots.front().get_dynamics().nv();
  int start = 0;

  for (int i = 0; i < n; ++i) {
    const VectorX<T>& x0 = stacked_decision_vars.segment(start, nx);
    start += nvars;
    const VectorX<T>& x1 = stacked_decision_vars.segment(start, nx);

    double dt = breaks.at(i+1) - breaks.at(i);
    VectorX<T> vdot = (x1.tail(nv) - x0.tail(nv)) / dt;
    knots.at(i).SetVDot(vdot);
    knots.at(i).Update<T>(stacked_decision_vars.segment(i * nx,  nvars));
  }
}

template void Timeline::Update(const VectorX<double> stacked_decision_vars);
template void Timeline::Update(const VectorX<AutoDiffXd> stacked_decision_vars);
}