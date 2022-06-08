#include "swing_foot_params_sender.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using std::vector;

SwingFootParamsSender::SwingFootParamsSender(int n_knot) :
 n_knot_(n_knot) {
  this->DeclareVectorInputPort(
      "raw params", BasicVector<double>(1 + 3*n_knot + 6));
  this->DeclareAbstractOutputPort(
      "lcmt_swing_foot_spline_params", dairlib::lcmt_swing_foot_spline_params(),
      &SwingFootParamsSender::CalcOutput);
}

void SwingFootParamsSender::CalcOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_swing_foot_spline_params* output) const {
  const auto& data = this->EvalVectorInput(context, 0)->get_value();
  output->n_knot = n_knot_;
  output->knot_xyz = vector<vector<double>>(n_knot_, vector<double>(3));
  for (int i = 0; i < n_knot_; i++) {
    for (int j = 0; j < 3; j++) {
      output->knot_xyz[i].at(j) = data(1 + 3*i + j);
    }
  }

  int s = n_knot_ * 3 + 1;
  for (int i = 0; i < 3; i++) {
    output->swing_foot_vel_initial[i] = data(s+i);
    output->swing_foot_vel_final[i] = data(s+i+3);
  }
}

}
}