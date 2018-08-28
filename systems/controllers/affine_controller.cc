#include "systems/controllers/affine_controller.h"
#include "systems/controllers/linear_controller.h"
namespace dairlib{
namespace systems{

AffineController::AffineController(int num_positions, int num_velocities,
                                   int num_inputs, VectorXd u_des, VectorXd x_des) {

  output_input_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(num_positions, num_velocities,
                           num_inputs)).get_index();

  config_input_port_ = this->DeclareVectorInputPort(
      LinearConfig(num_positions + num_velocities, num_inputs)).get_index();

  this->DeclareVectorOutputPort(TimestampedVector<double>(num_inputs),
      &AffineController::CalcControl);

  u_des_ = u_des;
  x_des_ = x_des;

}

void AffineController::CalcControl(const Context<double>& context,
                                  TimestampedVector<double>* control) const {
    const OutputVector<double>* output = (OutputVector<double>*)
        this->EvalVectorInput(context, output_input_port_);

    const LinearConfig* config = dynamic_cast<const LinearConfig*>(
        this->EvalVectorInput(context, config_input_port_));
    VectorXd u = config->GetK() *
        (x_des_ - output->GetState()) + u_des_;

    control->SetDataVector(u);
    control->set_timestamp(output->get_timestamp());
}


}
}
