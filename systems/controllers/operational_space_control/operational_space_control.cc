//TODO(yminchen):
// In constructor of OSC, you call checkConstraintSettings() function to check
// that the user set the constraint correctly.
// This function takes in RBT and check the following things:
// - size of body_index_, pt_on_body_ and mu_ are the same
// - 

//TODO(yminchen):
// In constructor of OSC, you call checkCostSettings() function to check
// that the user set the cost correctly.
// This function takes in RBT and check the following things:
// - 


#include "systems/controllers/operational_space_control/operational_space_control.h"

using std::vector;
using std::string;
using Eigen::VectorXd;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib {
namespace systems {


OperationalSpaceControl::OperationalSpaceControl(
  OscTrackingDataSet tracking_data_set) {
  this->set_name("OSC");

  // TODO: construct traj_name_to_port_index_map_
  vector<OscTrackingData*> tracking_data_vec = tracking_data_set.GetAllTrackingData();
  for(auto tracking_data : tracking_data_vec){
    string traj_name tracking_data->name_;
    int port_index;
    if(tracking_data->traj_has_exp_){
      port_index = this->DeclareAbstractInputPort(traj_name,
                drake::Value<ExponentialPlusPiecewisePolynomial<double>> {}).get_index();
    } else {
      port_index = this->DeclareAbstractInputPort(traj_name,
                drake::Value<PiecewisePolynomial<double>> {}).get_index();
    }
    name_to_index_map[traj_name] = port_index;
  }

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                  OutputVector<double>(num_positions,
                                       num_velocities,
                                       num_inputs)).get_index();
  FSM_port_ = this->DeclareVectorInputPort(
                BasicVector<double>(1)).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &OperationalSpaceControl::CalcOptimalInput);

  //
  checkConstraintSettings();
  checkCostSettings();
}

// I don't think we need this
// API for the user to add input ports if they create the traj source themselves
void OperationalSpaceControl::AddTrackingDataInputPort(string name){
  // you can call num_input_ports() for checking
}

void OperationalSpaceControl::CalcOptimalInput(
  const drake::systems::Context<double>& context,
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


