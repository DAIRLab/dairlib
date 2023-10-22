#pragma once


// Elevation mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

// Drake
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace perception {

struct sensor_pose_params {
  std::string sensor_name_;
  std::string sensor_parent_body_;
  drake::math::RigidTransformd sensor_pose_in_parent_body_;
};

struct elevation_map_update_params {
  drake::systems::TriggerType map_update_type_ = drake::systems::TriggerType::kPeriodic;
  double map_update_rate_hz_ = 30;
};

struct elevation_mapping_params {
  std::vector<sensor_pose_params> sensor_poses;
  elevation_map_update_params update_params;
  std::string base_frame_name;
  Eigen::Vector3d track_point;
};

class ElevationMappingSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElevationMappingSystem);

  ElevationMappingSystem(const drake::multibody::MultibodyPlant<double>& plant,
                         drake::systems::Context<double>* context,
                         elevation_mapping_params params);

  const drake::systems::InputPort<double>& get_input_port_pointcloud(
      const std::string& sensor_name) const {
    DRAKE_DEMAND(input_ports_pcl_.count(sensor_name) == 1);
    return get_input_port(input_ports_pcl_.at(sensor_name));
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_covariance() const {
    return get_input_port(input_port_pose_covariance_);
  }

  void AddSensorPreprocessor(
      const std::string& sensor_name,
      std::unique_ptr<elevation_mapping::SensorProcessorBase> processor) {
    DRAKE_DEMAND(sensor_poses_.count(sensor_name) == 1);
    sensor_preprocessors_.insert({sensor_name, std::move(processor)});
  }

 private:

  drake::systems::EventStatus ElevationMapUpdateEvent(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  // multibody
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::Body<double>& robot_base_;
  drake::systems::Context<double>* context_;
  std::map<std::string, sensor_pose_params> sensor_poses_;
  const Eigen::Vector3d track_point_; // point in the base frame to pin the map


  // ports
  std::map<std::string, drake::systems::InputPortIndex> input_ports_pcl_;
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_pose_covariance_;
  drake::systems::OutputPortIndex output_port_elevation_map_;

  // states
  drake::systems::AbstractStateIndex elevation_map_state_index_;
  drake::systems::AbstractStateIndex motion_updater_state_index_;
  std::map<std::string,
           drake::systems::DiscreteStateIndex> sensor_prev_update_time_indices_;

  std::map<std::string,
           std::unique_ptr<elevation_mapping::SensorProcessorBase>> sensor_preprocessors_;
};

}
}
