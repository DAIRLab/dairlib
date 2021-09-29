#include "examples/cartpole/lqr.h"

namespace dairlib:: systems {

LQR::LQR(int num_positions, int num_velocities,
    int num_inputs, const VectorXd& x_des, const MatrixXd& K) :
    x_des_(x_des), K_ (K) {

    this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  this->DeclareVectorOutputPort("u", TimestampedVector<double>(num_inputs),
                                &LQR::CalcControl);
}

void LQR::CalcControl(const Context<double>& context,
    TimestampedVector<double>* control) const {

  const OutputVector<double>* output =
      (OutputVector<double>*)this->EvalVectorInput(context, 0);

  VectorXd x_tilde = x_des_ - output->GetState();
  VectorXd u = K_ * (x_tilde);

  control->SetDataVector(u);
  control->set_timestamp(output->get_timestamp());
}
}