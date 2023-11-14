#pragma once
#include "multibody/stepping_stone_utils.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace dairlib::perceptive_locomotion {

class HikingSimDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HikingSimDiagram)
  HikingSimDiagram(const std::variant<
      std::string, multibody::SquareSteppingStoneList> &terrain,
                   const std::string &camera_pose_yaml);

  const drake::systems::InputPort<double> &get_input_port_actuation() const {
    return get_input_port(input_port_control_);
  }
  const drake::systems::InputPort<double> &get_input_port_radio() const {
    return get_input_port(input_port_radio_);
  }
  const drake::systems::OutputPort<double> &get_output_port_state() const {
    return get_output_port(output_port_state_);
  }
  const drake::systems::OutputPort<double> &get_output_port_state_lcm() const {
    return get_output_port(output_port_state_lcm_);
  }
  const drake::systems::OutputPort<double> &get_output_port_cassie_out() const {
    return get_output_port(output_port_cassie_out_);
  }
  const drake::systems::OutputPort<double> &get_output_port_lcm_radio() const {
    return get_output_port(output_port_lcm_radio_);
  }
  const drake::systems::OutputPort<double> &get_output_port_scene_graph_query()
  const {
    return get_output_port(output_port_scene_graph_query_);
  }
  const drake::systems::OutputPort<double>& get_output_port_depth_image()
  const {
    return get_output_port(output_port_depth_image_);
  }

  const drake::geometry::DrakeVisualizer<double> &AddDrakeVisualizer(
      drake::systems::DiagramBuilder<double> *builder
  ) const;

  [[nodiscard]] drake::multibody::MultibodyPlant<double> &get_plant() const {
    return *plant_;
  }

  /// returns the q, v IK solution as a pair
  std::pair<Eigen::VectorXd, Eigen::VectorXd> SetPlantInitialConditionFromIK(
      const drake::systems::Diagram<double> *parent_diagram,
      drake::systems::Context<double> *parent_context,
      const Eigen::Vector3d &pelvis_vel,
      double foot_spread,
      double height
  ) const;

  const drake::systems::sensors::CameraInfo&
  get_depth_camera_info(std::string sensor_name) {
    return GetDowncastSubsystemByName<drake::systems::sensors::RgbdSensor>(
        sensor_name
    ).depth_camera_info();
  }

  void SetPlantInitialCondition(
      const drake::systems::Diagram<double> *parent_diagram,
      drake::systems::Context<double> *parent_context,
      const Eigen::VectorXd &q,
      const Eigen::VectorXd &v
  ) const;

 private:

  const std::string urdf_;

  drake::multibody::MultibodyPlant<double> *plant_;
  drake::geometry::SceneGraph<double> *scene_graph_;

  drake::systems::InputPortIndex input_port_control_;
  drake::systems::InputPortIndex input_port_radio_;
  drake::systems::OutputPortIndex output_port_state_;
  drake::systems::OutputPortIndex output_port_state_lcm_;
  drake::systems::OutputPortIndex output_port_cassie_out_;
  drake::systems::OutputPortIndex output_port_lcm_radio_;
  drake::systems::OutputPortIndex output_port_scene_graph_query_;
  drake::systems::OutputPortIndex output_port_depth_image_;
};

}