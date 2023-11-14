#pragma once

// dairlib
#include "camera_utils.h"
#include "common/find_resource.h"
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

// Drake
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_io.h"

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

struct elevation_mapping_params_io {
  std::vector<std::map<std::string, std::string>> sensor_poses;
  std::string map_update_trigger_type;
  std::string base_frame_name;
  std::vector<double> track_point;
  double map_update_rate_hz;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(sensor_poses));
    a->Visit(DRAKE_NVP(map_update_trigger_type));
    a->Visit(DRAKE_NVP(base_frame_name));
    a->Visit(DRAKE_NVP(track_point));
    a->Visit(DRAKE_NVP(map_update_rate_hz));
  }

  static elevation_mapping_params ReadElevationMappingParamsFromYaml(
      const std::string& filename) {
    elevation_mapping_params params_out;
    auto params_io = drake::yaml::LoadYamlFile<elevation_mapping_params_io>(
        FindResourceOrThrow(filename)
    );
    params_out.sensor_poses.clear();
    for (const auto& pose_info : params_io.sensor_poses) {
      params_out.sensor_poses.push_back({
        pose_info.at("sensor_name"),
        pose_info.at("sensor_parent_body"),
        camera::ReadCameraPoseFromYaml(FindResourceOrThrow(
            pose_info.at("sensor_pose_in_parent_body_yaml")
        ))
      });
    }
    DRAKE_DEMAND(params_io.map_update_trigger_type == "periodic" or
                 params_io.map_update_trigger_type == "forced");
    if (params_io.map_update_trigger_type == "forced") {
      params_out.update_params.map_update_type_ =
          drake::systems::TriggerType::kForced;
    } else {
      params_out.update_params.map_update_type_ =
          drake::systems::TriggerType::kPeriodic;
      params_out.update_params.map_update_rate_hz_ =
          params_io.map_update_rate_hz;
    }
    DRAKE_DEMAND(params_io.track_point.size() == 3);
    params_out.track_point = Eigen::Vector3d::Map(params_io.track_point.data());
    params_out.base_frame_name = params_io.base_frame_name;
    return params_out;
  };
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
  const drake::systems::OutputPort<double>& get_output_port_map() const {
    return get_output_port(output_port_elevation_map_);
  }
  const drake::systems::OutputPort<double>& get_output_port_grid_map() const {
    return get_output_port(output_port_grid_map_);
  }

  void AddSensorPreProcessor(
      const std::string& sensor_name,
      std::shared_ptr<elevation_mapping::SensorProcessorBase> processor);

 private:

  drake::systems::EventStatus ElevationMapUpdateEvent(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyGridMap(const drake::systems::Context<double>& context,
                   grid_map::GridMap* grid_map) const;

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
  drake::systems::OutputPortIndex output_port_grid_map_;

  // states
  drake::systems::AbstractStateIndex elevation_map_state_index_;
  drake::systems::AbstractStateIndex motion_updater_state_index_;
  std::map<std::string,
           drake::systems::DiscreteStateIndex> sensor_prev_update_time_indices_;

  std::map<std::string,
           std::shared_ptr<elevation_mapping::SensorProcessorBase>> sensor_preprocessors_;
};

}
}
