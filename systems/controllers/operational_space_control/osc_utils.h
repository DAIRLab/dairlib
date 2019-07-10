#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

//TODO(yminchen): You can actually set the names of systems and connect them
int ConnectPortsForAllTrackingData(OperationalSpaceControl* osc,
                                   drake::systems::DiagramBuilder<double> & builder) {
  OscTrackingDataSet data_set = osc->GetTrackingDataSet();
  std::vector<OscTrackingData> tracking_data_vec = data_set.GetAllTrackingData();
  for (int i = 0; i < tracking_data_vec->size(); i++) {
    string traj_name = tracking_data_vec[i]->traj_name_;

    std::vector<systems::System<double>*> system_vec = builder.GetMutableSystems();
    bool connect_successfully = false;
    // TODO: you can actually get the port name
    for (auto system : system_vec) {
      if (traj_name.compare(system->get_name()) == 0) {
        DRAKE_DEMAND(traj_source->num_output_ports() == 1);
        builder.Connect(traj_source->get_output_port(0),
                        osc->get_tracking_data_input_port(traj_name));
        connect_successfully = true;
        break;
      }
    }
    DRAKE_DEMAND(connect_successfully);
  }
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
