#include "systems/controllers/linear_controller.h"

namespace dairlib{
namespace systems{

LinearController::LinearController(int num_positions, int num_velocities,
                                   int num_inputs) {

  output_input_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(num_positions, num_velocities,
                           num_inputs)).get_index();

  config_input_port_ = this->DeclareVectorInputPort(
      LinearConfig(num_positions + num_velocities, num_inputs)).get_index();

  this->DeclareVectorOutputPort(TimestampedVector<double>(num_inputs),
      &LinearController::CalcControl);
}

void LinearController::CalcControl(const Context<double>& context,
                                  TimestampedVector<double>* control) const {
    const OutputVector<double>* output = (OutputVector<double>*)
        this->EvalVectorInput(context, output_input_port_);

    const LinearConfig* config = dynamic_cast<const LinearConfig*>(
        this->EvalVectorInput(context, config_input_port_));
    VectorXd u = config->GetK() *
        (config->GetDesiredState() - output->GetState());

    std::cout << "xd = " << config->GetDesiredState().transpose() << std::endl;
    std::cout << "x = " << output->GetState().transpose() << std::endl;
    std::cout << "u = " << u.transpose() << std::endl;
//    std::cout << "u_fb = " << output->GetEfforts().transpose() << std::endl;


  control->SetDataVector(u);
    control->set_timestamp(output->get_timestamp());
}


}
}