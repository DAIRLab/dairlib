#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

using std::vector;
using drake::systems::System;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {
namespace controllers {

//TODO(yminchen): You can actually set the names of systems and connect them

// Requiremnt:
// The name of traj source block and its output port must have the same name as
// `TrackingData.name_`
void ConnectPortsForNonConstTraj(OperationalSpaceControl* osc,
                                 drake::systems::DiagramBuilder<double> & builder) {
  OscTrackingDataSet data_set = osc->GetTrackingDataSet();
  vector<OscTrackingData> tracking_data_vec = data_set.GetAllTrackingData();
  for (auto tracking_data : tracking_data_vec) {
    string traj_name = tracking_data->traj_name_;

    bool connect_successfully = false;
    vector<System<double>*> system_vec = builder.GetMutableSystems();
    // Find trajectory source block
    for (auto system : system_vec) {
      if (traj_name.compare(system->get_name()) == 0) {
        // Find the correspond output port
        for (int i = 0; i < system->num_output_ports(); i++) {
          if (traj_name.compare(system->get_output_port(i).get_name()) == 0) {
            builder.Connect(system->get_output_port(i),
                            osc->get_tracking_data_input_port(traj_name));
            connect_successfully = true;
            break;
          }
        }  // end for (ports)
      }
      if (connect_successfully) break;
    }  // end for (leaf systems)
    DRAKE_DEMAND(connect_successfully);
  }  // end for (OscTrackingData's)
}

void AssignConstTrajToInputPorts(drake::systems::Diagram<double> diagram,
                                 drake::systems::Context<double>* diagram_context) {
  OscTrackingDataSet data_set = osc->GetTrackingDataSet();
  vector<OscTrackingData> tracking_data_vec = data_set.GetAllTrackingData();
  for (auto tracking_data : tracking_data_vec) {
    if (!tracking_data->traj_is_const_) continue;

    string traj_name = tracking_data->traj_name_;

    bool fix_port_value_successfully = false;
    vector<const System<double>*> system_vec = diagram->GetSystems();
    // Find OSC block
    for (auto system : system_vec) {
      if (system->get_name().compare("OSC") == 0) {
        auto & osc_conext = diagram->GetMutableSubsystemContext(system,
                            diagram_context);
        // Find the correspond output port
        for (int i = 0; i < system->num_input_ports(); i++) {
          if (traj_name.compare(system->get_input_port(i).get_name()) == 0) {
            osc_conext.FixInputPort(i /*system->get_input_port(i).get_index()*/,
                                    drake::Value<PiecewisePolynomial<double>>(
                                      tracking_data->fixed_position_));
            fix_port_value_successfully = true;
            break;
          }
        }  // end for (ports)
      }
      if (fix_port_value_successfully) break;
    }  // end for (leaf systems)
    DRAKE_DEMAND(connect_successfully);
  }  // end for (OscTrackingData's)
}


// Convert rotational matrix to Eular angle (roll pitch yaw)
void RotationalMatrixToRollPitchYaw(
  const Eigen::MatrixXd& mat, double* roll, double* pitch, double* yaw);

// Convert Quaternion to Eular angle (roll pitch yaw)
void QuaternionToRollPitchYaw(
  const Eigen::Quaterniond& q, double* roll, double* pitch, double* yaw);


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
