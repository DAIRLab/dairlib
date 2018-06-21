#include "systems/controllers/clqr_controller.h"

namespace dairlib{
namespace systems{

ClqrController::ClqrController(int num_positions, int num_velocities, int num_inputs)
{
    num_positions_ = num_positions;
    num_velocities_ = num_velocities;
    num_states_ = num_positions_ + num_velocities_;
    num_inputs_ = num_inputs;

    input_state_port_index_ = this->DeclareVectorInputPort(BasicVector<double>(num_states_)).get_index();
    input_desired_port_index = this->DeclareVectorInputPort(BasicVector<double>(num_states_)).get_index();

}

    







