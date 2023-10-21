#include "systems/perception/elevation_mapping_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace perception {

using systems::OutputVector;

using drake::systems::State;
using drake::systems::Context;
using drake::systems::TriggerType;
using drake::multibody::MultibodyPlant;

using elevation_mapping::ElevationMap;
using elevation_mapping::RobotMotionMapUpdater;

ElevationMappingSystem::ElevationMappingSystem(
    const MultibodyPlant<double>& plant,
    Context<double>* context,
    const std::vector<sensor_pose_params>& sensor_poses,
    elevation_map_update_params update_params) :
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
  ElevationMap map;
  map.setGeometry(length, resolution, position);

  auto model_value_map = drake::Value<ElevationMap>(map);
  auto model_value_updater = drake::Value<RobotMotionMapUpdater>();
  elevation_map_state_index_ = DeclareAbstractState(model_value_map);
  motion_updater_state_index_ = DeclareAbstractState(model_value_updater);

  output_port_elevation_map_ = DeclareStateOutputPort(
      "elevation_map", elevation_map_state_index_
  ).get_index();

  // create update events
  switch (update_params.map_update_type_) {
    case TriggerType::kForced:
      DeclareForcedUnrestrictedUpdateEvent(
          &ElevationMappingSystem::ElevationMapUpdateEvent
    );
      break;
    case TriggerType::kPeriodic:
      DRAKE_DEMAND(update_params.map_update_rate_hz_ > 0);
      DeclarePeriodicUnrestrictedUpdateEvent(
          1.0 / update_params.map_update_rate_hz_, 0,
          &ElevationMappingSystem::ElevationMapUpdateEvent
      );
      break;
    default:
      throw std::runtime_error(
          "Elevation map update type must be kPeriodic or kForced");
  }
  switch (update_params.pose_update_type_) {
    case TriggerType::kForced:
      DeclareForcedUnrestrictedUpdateEvent(
          &ElevationMappingSystem::RobotPoseUpdateEvent
      );
      break;
    case TriggerType::kPeriodic:
      DRAKE_DEMAND(update_params.pose_update_rate_hz_ > 0);
      DeclarePeriodicUnrestrictedUpdateEvent(
          1.0 / update_params.pose_update_rate_hz_, 0,
          &ElevationMappingSystem::RobotPoseUpdateEvent
      );
      break;
    default:
      throw std::runtime_error(
          "Robot pose update type must be kPeriodic or kForced");
  }
}

drake::systems::EventStatus ElevationMappingSystem::ElevationMapUpdateEvent(
    const Context<double>& context, State<double>* state) const {

  // Get any new pointclouds from input ports
  std::map<std::string, elevation_mapping::PointCloudType::Ptr> new_pointclouds{};
  for (const auto& [name, input_port_idx_pcl] : input_ports_pcl_) {
    double prev_pointcloud_stamp = state->get_discrete_state(
        sensor_prev_update_time_indices_.at(name)
    ).value()(0);
    auto pointcloud = EvalAbstractInput(
        context, input_port_idx_pcl
    )->get_value<elevation_mapping::PointCloudType::Ptr>();
    if (pointcloud->header.stamp * 1e-6 != prev_pointcloud_stamp) {
      new_pointclouds.insert({name, pointcloud});
    }
  }

  if (new_pointclouds.empty()) {
    return drake::systems::EventStatus::DidNothing();
  }


  // TODO (@Brian-Acosta) step by step from ros ElevationMapping :
  // 1. Get the robot base pose and covariance
  // 2. apply sensor preprocessor to pointcloud
  // 3. updateMapLocation
  // 4. updatePrediction
  // 5. maybe clean up the map
  // 6. add the pointcloud to the map
  // 7. maybe publish the map

  return drake::systems::EventStatus::Succeeded();
}


}
}