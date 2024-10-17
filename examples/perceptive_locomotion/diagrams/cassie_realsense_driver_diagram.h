#pragma once

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/plant/multibody_plant.h"

#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/pointcloud/lcm_to_pcl_point_cloud.h"
#include "systems/perception/perceptive_locomotion_preprocessor.h"
#include "systems/perception/realsense/single_rs_interface.h"
#include "systems/perception/realsense/realsense_point_cloud_subscriber.h"
#include "systems/perception/realsense/realsense_image_pair_subscriber.h"


#include "examples/perceptive_locomotion/cassie_perception_utils.h"

namespace dairlib {
namespace perceptive_locomotion {

/** diagram for elevation mapping with LCM point clouds */
class CassieRealSenseDriverDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieRealSenseDriverDiagram);
  CassieRealSenseDriverDiagram(const std::string& params_yaml);

  void InitializeElevationMap(const Eigen::VectorXd& robot_state,
                              drake::systems::Context<double>* root_context) const;

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_robot_state_);
  }

  const drake::systems::InputPort<double>& get_input_port_contact() const {
    DRAKE_DEMAND(elevation_mapping_system_->has_contacts());
    return get_input_port(input_port_contact_);
  }

  const drake::systems::OutputPort<double>& get_output_port_state() const {
    return get_output_port(output_port_state_);
  }
  
  const drake::systems::OutputPort<double>& get_output_port_grid_map() const {
    return get_output_port(output_port_grid_map_);
  }

  drake::lcm::DrakeLcm* lcm() {return &lcm_local_;}

  const drake::multibody::MultibodyPlant<double>& plant() {
    return plant_;
  }

  void start_rs() { realsense_.Start(); }

  void stop_rs() { realsense_.Stop(); }

 private:
  // plant
  drake::multibody::MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  // messaging
  drake::lcm::DrakeLcm lcm_local_{"udpm://239.255.76.67:7667?ttl=0"};

  // realsense
  rs2_systems::SingleRSInterface realsense_{};
  perception::RealsensePointCloudSubscriber<pcl::PointXYZRGBConfidenceRatio>* point_cloud_subscriber_;
  perception::RealsenseImagePairSubscriber* image_pair_subscriber_;


  // elevation_mapping
  std::shared_ptr<perception::PerceptiveLocomotionPreprocessor> sensor_processor_;
  perception::elevation_mapping_params elevation_mapping_params_;
  perception::ElevationMappingSystem* elevation_mapping_system_;

  drake::systems::InputPortIndex input_port_robot_state_;
  drake::systems::InputPortIndex input_port_contact_;
  drake::systems::OutputPortIndex output_port_grid_map_;
  drake::systems::OutputPortIndex output_port_state_;


};

}
}