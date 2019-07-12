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
using Eigen::MatrixXd;

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;

namespace dairlib {
namespace systems {
namespace controllers {


OperationalSpaceControl::OperationalSpaceControl(
  OscTrackingDataSet* tracking_data_set,
  RigidBodyTree<double>* tree_with_springs,
  RigidBodyTree<double>* tree_without_springs) :
  tree_with_springs_(tree_with_springs),
  tree_without_springs_(tree_without_springs),
  tracking_data_set_(tracking_data_set) {
  this->set_name("OSC");

  int n_q = tree_with_springs->get_num_positions();
  int n_v = tree_with_springs->get_num_velocities();
  int n_u = tree_with_springs->get_num_actuators();

  // TODO: construct traj_name_to_port_index_map_
  vector<OscTrackingData*> tracking_data_vec =
    tracking_data_set->GetAllTrackingData();
  for (auto tracking_data : tracking_data_vec) {
    string traj_name = tracking_data->GetName();
    int port_index;
    if (tracking_data->DoesTrajHasExp()) {
      port_index = this->DeclareAbstractInputPort(traj_name,
                   drake::Value<ExponentialPlusPiecewisePolynomial<double>> {}).get_index();
    } else {
      port_index = this->DeclareAbstractInputPort(traj_name,
                   drake::Value<PiecewisePolynomial<double>> {}).get_index();
    }
    traj_name_to_port_index_map_[traj_name] = port_index;
  }

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                  OutputVector<double>(n_q, n_v, n_u)).get_index();
  FSM_port_ = this->DeclareVectorInputPort(
                BasicVector<double>(1)).get_index();
  this->DeclareVectorOutputPort(TimestampedVector<double>(n_u),
                                &OperationalSpaceControl::CalcOptimalInput);

  //
  checkConstraintSettings();
  checkCostSettings();
}

// I don't think we need this
// API for the user to add input ports if they create the traj source themselves
void OperationalSpaceControl::AddTrackingDataInputPort(string name) {
  // you can call num_input_ports() for checking
}

void OperationalSpaceControl::CalcOptimalInput(
  const drake::systems::Context<double>& context,
  systems::TimestampedVector<double>* control) const {
  // Read in current state and simulation time
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd current_state = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();
  double current_sim_time = static_cast<double>(timestamp);




  VectorXd u;
  // Assign the control input
  control->SetDataVector(u);
  control->set_timestamp(robot_output->get_timestamp());
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib


