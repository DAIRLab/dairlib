
//dairlib
#include "multibody/multibody_utils.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace perception {

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

using systems::OutputVector;

using drake::systems::State;
using drake::systems::Context;
using drake::systems::TriggerType;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::MultibodyPlant;

using grid_map::GridMap;
using elevation_mapping::ElevationMap;
using elevation_mapping::PointCloudType;
using elevation_mapping::RobotMotionMapUpdater;
using elevation_mapping::SensorProcessorBase;

ElevationMappingSystem::ElevationMappingSystem(
    const MultibodyPlant<double>& plant,
    Context<double>* context,
    elevation_mapping_params params) :
    plant_(plant), context_(context),
    robot_base_(plant.GetBodyByName(params.base_frame_name)),
    track_point_(params.track_point) {

  // configure sensors
  drake::Vector1d prev_time_model_vector{-1};
  for (const auto& pose_param : params.sensor_poses) {
    DRAKE_DEMAND(plant_.HasBodyNamed(pose_param.sensor_parent_body_));
    DRAKE_DEMAND(sensor_poses_.count(pose_param.sensor_name_) == 0);
    sensor_poses_.insert({pose_param.sensor_name_, pose_param});
    input_ports_pcl_.insert({pose_param.sensor_name_, DeclareAbstractInputPort(
        "Point_cloud_" + pose_param.sensor_name_,
        drake::Value<PointCloudType::Ptr>()).get_index()
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
  double resolution = 0.025;
  ElevationMap map;
  map.setGeometry(length, resolution, track_point_.head<2>());

  //TODO (@Brian-Acosta) how to handle map and motion updater initialization?

  auto model_value_map = drake::Value<ElevationMap>(map);
  auto model_value_updater = drake::Value<RobotMotionMapUpdater>();
  elevation_map_state_index_ = DeclareAbstractState(model_value_map);
  motion_updater_state_index_ = DeclareAbstractState(model_value_updater);

  output_port_elevation_map_ = DeclareStateOutputPort(
      "elevation_map", elevation_map_state_index_
  ).get_index();

  output_port_grid_map_ = DeclareAbstractOutputPort(
      "grid_map", &ElevationMappingSystem::CopyGridMap
  ).get_index();

  // create update event
  auto& update_params = params.update_params;
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
}

void ElevationMappingSystem::AddSensorPreProcessor(
    const std::string& sensor_name,
    std::shared_ptr<SensorProcessorBase> processor) {

  DRAKE_DEMAND(sensor_poses_.count(sensor_name) == 1);
  auto processor_derived = dynamic_cast<PerceptiveLocomotionPreprocessor*>(
      processor.get()
  );
  if (processor_derived != nullptr) {
    // processor must share the plant context in order for state info to be
    // propagated correctly
    DRAKE_DEMAND(processor_derived->context() == context_);
  }
  sensor_preprocessors_.insert({sensor_name, processor});
}

drake::systems::EventStatus ElevationMappingSystem::ElevationMapUpdateEvent(
    const Context<double>& context, State<double>* state) const {

  // Get any new pointclouds from input ports
  std::map<std::string, PointCloudType::Ptr> new_pointclouds{};
  for (const auto& [name, input_port_idx_pcl] : input_ports_pcl_) {

    // Get the timestamp of the previous pointcloud
    double prev_pointcloud_stamp = state->get_discrete_state(
        sensor_prev_update_time_indices_.at(name)
    ).value()(0);

    auto pointcloud = EvalAbstractInput(
        context, input_port_idx_pcl
    )->get_value<PointCloudType::Ptr>();

    // Only do anything if the timestamp has been updated
    if (pointcloud->header.stamp * 1e-6 > prev_pointcloud_stamp) {
      new_pointclouds.insert({name, pointcloud});

      // update the timestamp of the previous point cloud seen from this sensor
      state->get_mutable_discrete_state(
          sensor_prev_update_time_indices_.at(name)
      ).set_value(drake::Vector1d::Constant(pointcloud->header.stamp * 1e-6));
    }
  }

  // Skip the rest of this function if no new point clouds
  if (new_pointclouds.empty()) {
    return drake::systems::EventStatus::DidNothing();
  }

  // Get the elevation map
  auto& map = state->get_mutable_abstract_state<ElevationMap>(
      elevation_map_state_index_
  );
  auto& motion_updater = state->get_mutable_abstract_state<RobotMotionMapUpdater>(
      motion_updater_state_index_
  );

  // 1. Get the robot base pose and covariance
  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_)
  );
  VectorXd q_v = robot_output->GetState();
  double timestamp = robot_output->get_timestamp();

  MatrixXd pose_covariance = EvalVectorInput(
      context, input_port_pose_covariance_
  )->get_value();
  pose_covariance.resize(6,6);
  multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, q_v, context_);
  const auto base_pose = plant_.EvalBodyPoseInWorld(*context_, robot_base_);

  // 2. Update the map location
  Vector3d track_point_in_world = base_pose * track_point_;
  map.move(track_point_in_world.head<2>());

  // 3. Apply prediction step
  motion_updater.update(map, base_pose, pose_covariance, timestamp);

  // 4. cleanup the map
  //  map.clear();

  // 5. add the point clouds to the map
  for (const auto& [name, cloud] : new_pointclouds) {
    // allocate data for processing
    Eigen::VectorXf measurement_variances;
    PointCloudType::Ptr pc_processed(new PointCloudType);

    // TODO (@Brian-Acosta) does it make sense to propogate the base variance
    //  when we add non-base parent frames?
    const auto X_WP = plant_.EvalBodyPoseInWorld(
        *context_,
        plant_.GetBodyByName(sensor_poses_.at(name).sensor_parent_body_)
    );
    const auto& X_PS =  sensor_poses_.at(name).sensor_pose_in_parent_body_;

    // apply preprocessor
    sensor_preprocessors_.at(name)->process(
        cloud,
        pose_covariance,
        pc_processed,
        measurement_variances,
        X_PS, X_WP
    );

    // add points to the map
    map.add(pc_processed, measurement_variances, timestamp, X_WP * X_PS);
  }

  // TODO (@Brian-Acosta) decide how to go about fusing if needed
  //  map.fuseAll();
  return drake::systems::EventStatus::Succeeded();
}

void ElevationMappingSystem::CopyGridMap(
    const Context<double>& context, GridMap* grid_map) const {
  *grid_map = context.get_abstract_state<ElevationMap>(
      elevation_map_state_index_
  ).getRawGridMap();
}

}
}