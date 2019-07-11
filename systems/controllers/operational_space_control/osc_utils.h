#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

//TODO(yminchen): You can actually set the names of systems and connect them

// Requiremnt:
// The name of traj source block and its output port must have the same name as
// `TrackingData.name_`
int ConnectPortsForAllTrackingData(OperationalSpaceControl* osc,
                                   drake::systems::DiagramBuilder<double> & builder) {
  OscTrackingDataSet data_set = osc->GetTrackingDataSet();
  std::vector<OscTrackingData> tracking_data_vec = data_set.GetAllTrackingData();
  for (auto tracking_data : tracking_data_vec) {
    string traj_name = tracking_data->traj_name_;

    std::vector<systems::System<double>*> system_vec = builder.GetMutableSystems();
    bool connect_successfully = false;
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

// Convert rotational matrix to Eular angle (roll pitch yaw)
void RotationalMatrixToRollPitchYaw(
  const Eigen::MatrixXd& mat, double* roll, double* pitch, double* yaw);

// Convert Quaternion to Eular angle (roll pitch yaw)
void QuaternionToRollPitchYaw(
  const Eigen::Quaterniond& q, double* roll, double* pitch, double* yaw);


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
