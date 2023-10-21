#include "systems/perception/elevation_mapping_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace perception {

using systems::OutputVector;

using drake::systems::Context;
using drake::multibody::MultibodyPlant;

ElevationMappingSystem::ElevationMappingSystem(
    const MultibodyPlant<double>& plant,
    Context<double>* context,
    const std::vector<sensor_pose_params>& sensor_poses) :
    plant_(plant), context_(context) {

  // configure sensors
  drake::Vector1d prev_time_model_vector{-1};
  for (const auto& pose_param : sensor_poses) {
    DRAKE_DEMAND(plant_.HasBodyNamed(pose_param.sensor_parent_body_));
    DRAKE_DEMAND(sensor_poses_.count(pose_param.sensor_name_) == 0);
    sensor_poses_.insert({pose_param.sensor_name_, pose_param});
    input_ports_pcl_.insert({pose_param.sensor_name_, DeclareAbstractInputPort(
        "Point_cloud_" + pose_param.sensor_name_,
        drake::Value<elevation_mapping::PointCloudType::Ptr>()).get_index()
    });
    sensor_prev_update_time_indices_.insert(
        {pose_param.sensor_name_, DeclareDiscreteState(prev_time_model_vector)}
    );
  }

  // state input
  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators()
      )).get_index();

  // covariance of the floating base pose in column major order
  input_port_pose_covariance_ = DeclareVectorInputPort("cov", 36).get_index();

  // create the elevation map
  // TODO (@Brian-Acosta) expose the elevation mapping parameters
  grid_map::Length length(2.0, 2.0);
  grid_map::Position position(0.0, 0.0);
  double resolution = 0.25;
  elevation_mapping::ElevationMap map;
  map.setGeometry(length, resolution, position);

  // TODO (@Brian-Acosta) need to make elevation maps cloneable
  //  (still need mutexes?)
  auto model_value = drake::Value<elevation_mapping::ElevationMap>(map);
}

}
}