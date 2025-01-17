#include "constant_reference_system.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Context;

ConstantReferenceSystem::ConstantReferenceSystem(const MPCReference &reference)
  : reference_(reference) {
  DeclareAbstractOutputPort(
      "mpc_reference", reference, &ConstantReferenceSystem::CalcOutput);
}

void ConstantReferenceSystem::CalcOutput(
    const Context<double> &context, MPCReference *out) const {
  double time = context.get_time();
  double offset = time - reference_.knot_times_.front();

  *out = reference_;
  for (int i = 0; i < out->knot_times_.size(); ++i) {
    out->knot_times_.at(i) += offset;
  }

}

}