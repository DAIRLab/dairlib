#pragma once

// dairlib - perception
#include "systems/perception/elevation_mapping_system.h"
#include "systems/perception/simulated_hmap_point_server.h"
#include "examples/Cassie/cassie_state_estimator.h"
#include "geometry/convex_polygon_set.h"

// drake
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace perceptive_locomotion {

class NonRenderPerceptionModuleDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NonRenderPerceptionModuleDiagram);

  NonRenderPerceptionModuleDiagram(
      std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
      std::string elevation_mapping_params_yaml_path,
      std::map<std::string, drake::systems::sensors::CameraInfo> depth_sensor_info,
      geometry::ConvexPolygonSet polygons,
      std::string joint_offsets_yaml = "");

  const drake::systems::InputPort<double>& get_input_port_cassie_out() const {
    return get_input_port(input_port_cassie_out_);
  }
  const drake::systems::InputPort<double>& get_input_port_gt_state() const {
    return get_input_port(input_port_gt_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_state() const {
    return get_output_port(output_port_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_robot_output()
  const {
    return get_output_port(output_port_robot_output_);
  }
  const drake::systems::OutputPort<double>& get_output_port_elevation_map()
  const {
    return get_output_port(output_port_elevation_map_);
  }

  void InitializeEkf(drake::systems::Context<double>* root_context,
                     const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                     const Eigen::Vector3d& accel,
                     const Eigen::Vector3d& gyro) const;

  void InitializeEkf(drake::systems::Context<double>* root_context,
                     const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;

  static std::unique_ptr<NonRenderPerceptionModuleDiagram> Make(
      std::string elevation_mapping_params_yaml_path,
      std::map<std::string, drake::systems::sensors::CameraInfo> depth_sensor_info,
      geometry::ConvexPolygonSet polygon_terrain,
      std::string joint_offsets_yaml = "");

  void InitializeElevationMap(const Eigen::VectorXd& robot_state,
                              drake::systems::Context<double>* root_context) const;

 private:

  // State estimator parameters
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::map<std::string, double> joint_offsets_;
  systems::CassieStateEstimator* state_estimator_;

  perception::elevation_mapping_params elevation_mapping_params_;
  perception::ElevationMappingSystem* elevation_mapping_system_;

  drake::systems::InputPortIndex input_port_cassie_out_;
  drake::systems::InputPortIndex input_port_gt_state_;

  drake::systems::OutputPortIndex output_port_state_;
  drake::systems::OutputPortIndex output_port_robot_output_;
  drake::systems::OutputPortIndex output_port_elevation_map_;

  const int communication_delay_periods_ = 1;
  const double ekf_update_period_ = 0.0005;
};

} // namespace perceptive_locomotion
} // namespace dairlib
