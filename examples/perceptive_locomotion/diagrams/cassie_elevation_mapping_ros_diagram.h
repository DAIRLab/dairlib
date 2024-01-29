#pragma once

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/plant/multibody_plant.h"

#include "systems/ros/ros_interface_system.h"
#include "systems/ros/ros_subscriber_system.h"
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/pointcloud/ros_point_cloud2_receiver.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"


#include "examples/perceptive_locomotion/cassie_perception_utils.h"

namespace dairlib {
namespace perceptive_locomotion {

/** diagram for elevation mapping with ros point clouds on hardware */
class CassieElevationMappingRosDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieElevationMappingRosDiagram);
  CassieElevationMappingRosDiagram(const std::string& params_yaml,
                             const std::string& points_topic);

  void InitializeElevationMap(const Eigen::VectorXd& robot_state,
                              drake::systems::Context<double>* root_context) const;

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_robot_state_);
  }
  const drake::systems::InputPort<double>& get_input_port_contact() const {
    DRAKE_DEMAND(elevation_mapping_system_->has_contacts());
    return get_input_port(input_port_contact_);
  }

  drake::lcm::DrakeLcm* lcm() {return &lcm_local_;}

  const drake::multibody::MultibodyPlant<double>& plant() {
    return plant_;
  }

 private:
  // plant
  drake::multibody::MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  // messaging
  drake::lcm::DrakeLcm lcm_local_{"udpm://239.255.76.67:7667?ttl=0"};

  // elevation_mapping
  std::shared_ptr<perception::PerceptiveLocomotionPreprocessor> sensor_processor_;
  perception::elevation_mapping_params elevation_mapping_params_;
  perception::ElevationMappingSystem* elevation_mapping_system_;

  drake::systems::InputPortIndex input_port_robot_state_;
  drake::systems::InputPortIndex input_port_contact_;
  drake::systems::OutputPortIndex output_port_grid_map_;


};

}
}