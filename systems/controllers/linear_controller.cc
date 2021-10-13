#include "systems/controllers/linear_controller.h"

namespace dairlib {
namespace systems {

LinearController::LinearController(int num_positions, int num_velocities,
                                   int num_inputs) {
  output_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  config_input_port_ =
      this->DeclareVectorInputPort(
              "K, x_des, t", LinearConfig(num_positions + num_velocities, num_inputs))
          .get_index();

  this->DeclareVectorOutputPort("u", TimestampedVector<double>(num_inputs),
                                &LinearController::CalcControl);
}

void LinearController::CalcControl(const Context<double>& context,
                                   TimestampedVector<double>* control) const {
  const OutputVector<double>* output =
      (OutputVector<double>*)this->EvalVectorInput(context, output_input_port_);

  const auto* config = dynamic_cast<const LinearConfig*>(
      this->EvalVectorInput(context, config_input_port_));
  VectorXd x_tilde = config->GetDesiredState() - output->GetState();
  for (int i = 0; i < x_tilde.size(); ++i) {
    if (x_tilde[i] > kMaxError) {
      x_tilde[i] = kMaxError;
    } else if (x_tilde[i] < -kMaxError) {
      x_tilde[i] = -kMaxError;
    }
  }
  VectorXd u = config->GetK() * (x_tilde);

  control->SetDataVector(u);
  control->set_timestamp(output->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib