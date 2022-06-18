#include "osc_debug_parser.h"
#include "dairlib/lcmt_osc_output.hpp"

namespace dairlib::systems::controllers {

using drake::systems::BasicVector;
using dairlib::lcmt_osc_output;
using Eigen::Vector3d;

OscDebugParser::OscDebugParser(std::string traj_name_of_interest) :
traj_name_(traj_name_of_interest) {
  this->DeclareAbstractInputPort(
      "lcmt_osc_debug", drake::Value<lcmt_osc_output>{});
  error_y_idx_ = this->DeclareVectorOutputPort(
      "error_y", BasicVector<double>(3), &OscDebugParser::CopyErrorY)
      .get_index();
}

void OscDebugParser::CopyErrorY(const drake::systems::Context<double> &context,
                                drake::systems::BasicVector<double> *output) const {
  auto osc_debug = this->EvalAbstractInput(
      context, 0)->get_value<lcmt_osc_output>();

  auto it = std::find(osc_debug.tracking_data_names.begin(),
      osc_debug.tracking_data_names.end(), traj_name_);
  if (it != osc_debug.tracking_data_names.end()) {
    int idx = it - osc_debug.tracking_data_names.begin();
    output->set_value(Vector3d(osc_debug.tracking_data.at(idx).error_y.data()));
  } else {
    output->set_value(Vector3d::Zero());
  }
}
}