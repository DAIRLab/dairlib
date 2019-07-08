#include "systems/controllers/operational_space_control/operational_space_control.h"

using std::string;
using Eigen::VectorXd;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {

OperationalSpaceControl::OperationalSpaceControl(
  OscTrackingDataSet tracking_data_set,
  OscConstraintSettings constraint_settings) {

  // Construct input port
  // use a method, also in the beginning of the method, call num_input_ports()
  // make sure it's 0 so that your trajs are in the start of the indices.

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                  OutputVector<double>(num_positions,
                                       num_velocities,
                                       num_inputs)).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &OperationalSpaceControl::CalcOptimalInput);
}

void CalcOptimalInput(const drake::systems::Context<double>& context,
                      dairlab::systems::TimestampedVector<double>* control) const {
  // Read in current state and simulation time
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd currentState = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();
  double current_sim_time = static_cast<double>(timestamp);





  // Assign the control input
  control->SetDataVector(u);
  control->set_timestamp(robotOutput->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib


