#include "systems/controllers/affine_controller.h"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::Context;

AffineController::AffineController(int num_positions, int num_velocities,
                                   int num_efforts)
    : num_states_(num_positions + num_velocities), num_efforts_(num_efforts) {
  // Input port that corresponds to the state information
  input_port_info_index_ =
      this->DeclareVectorInputPort(
              OutputVector<double>(num_positions, num_velocities, num_efforts))
          .get_index();

  // Input port that corresponds to the parameters (constants and desired state)
  // of the controller
  input_port_params_index_ =
      this->DeclareVectorInputPort(AffineParams(num_states_, num_efforts_))
          .get_index();

  // Ouput port for the actuator efforts
  output_port_control_index_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(num_efforts_),
                                    &AffineController::CalcControl)
          .get_index();
}

void AffineController::CalcControl(const Context<double>& context,
                                   TimestampedVector<double>* control) const {
  const OutputVector<double>* info =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   input_port_info_index_);

  const AffineParams* params = dynamic_cast<const AffineParams*>(
      this->EvalVectorInput(context, input_port_params_index_));

  // Could use MatrixXd instead of auto, but that would force a copy.
  auto K = params->get_K();
  auto desired_state = params->get_desired_state();
  auto E = params->get_E();

  VectorXd u = K * (desired_state - info->GetState()) + E;

  control->SetDataVector(u);
  control->set_timestamp(info->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
