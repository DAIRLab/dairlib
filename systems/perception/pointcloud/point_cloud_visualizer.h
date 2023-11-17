#pragma once
#include <map>
#include "systems/perception/elevation_mapping_system.h"

#include "drake/geometry/meshcat.h"
#include "drake/perception/point_cloud.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace perception {

class PointCloudVisualizer : public drake::systems::LeafSystem<double> {
  PointCloudVisualizer(
      const drake::multibody::MultibodyPlant<double>& plant,
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      double update_rate,
      std::vector<sensor_pose_params> sensor_poses);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  };
  const drake::systems::InputPort<double>& get_input_port_point_cloud(
      const std::string& sensor_name) const {
    DRAKE_DEMAND(input_ports_pointcloud_.count(sensor_name) >= 1);
    return get_input_port(input_ports_pointcloud_.at(sensor_name));
  };
 private:
  drake::systems::InputPortIndex input_port_state_;
  std::map<std::string, drake::systems::InputPortIndex> input_ports_pointcloud_;
};

}
}