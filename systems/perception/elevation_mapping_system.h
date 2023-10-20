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

class ElevationMappingSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElevationMappingSystem);

  ElevationMappingSystem(const drake::multibody::MultibodyPlant<double>& plant,
                         drake::systems::Context<double>* context,
                         const std::vector<sensor_pose_params>& sensor_poses);

 private:

  drake::systems::EventStatus PeriodicUnrestrictedUpdateEvent(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  void CopyElevationMap(const drake::systems::Context<double>& context,
                        elevation_mapping::ElevationMap* map) const;

  // multibody
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  std::map<std::string, sensor_pose_params> sensor_poses_;

  // ports
  std::map<std::string, drake::systems::InputPortIndex> input_ports_pcl_;
  drake::systems::InputPortIndex input_port_robot_state_;
  drake::systems::InputPortIndex input_port_pose_covariance_;
  drake::systems::OutputPortIndex output_port_elevation_map_;

  // states
  drake::systems::AbstractStateIndex map_state_index_;

};

}
}
