#pragma once

#include <memory>
#include <utility>

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc_run/osc_running_gains.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/controllers/osc/osc_gains.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"

namespace dairlib {
namespace examples {
namespace controllers {

using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::JointSpaceTrackingData;

class OSCRunningControllerDiagram final
    : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OSCRunningControllerDiagram)

  /// @param[in] osc_gains_filename filepath containing the osc_running_gains.
  /// @param[in] osqp_settings filepath containing the osqp settings.
  OSCRunningControllerDiagram(drake::multibody::MultibodyPlant<double>& plant,
                              const OSCGains& osc_gains, const OSCRunningGains& osc_running_gains);

  /// @return the input port for the plant state.
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_index_);
  }

  /// @return the input port for the cassie_out struct (containing radio
  /// commands).
  const drake::systems::InputPort<double>& get_cassie_out_input_port() const {
    return this->get_input_port(cassie_out_input_port_index_);
  }

  /// @return the output port for the controller torques.
  const drake::systems::OutputPort<double>& get_control_output_port() const {
    return this->get_output_port(control_output_port_index_);
  }

  /// @return the output port for the failure status of the controller.
  const drake::systems::OutputPort<double>& get_controller_failure_output_port()
      const {
    return this->get_output_port(controller_failure_port_index_);
  }

 private:
  std::map<std::string, int> pos_map;
  std::map<std::string, int> vel_map;
  std::map<std::string, int> act_map;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      left_toe;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      left_heel;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      right_toe;
  std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
      right_heel;
  multibody::WorldYawViewFrame<double> view_frame;
  multibody::WorldPointEvaluator<double> left_toe_evaluator;
  multibody::WorldPointEvaluator<double> left_heel_evaluator;
  multibody::WorldPointEvaluator<double> right_toe_evaluator;
  multibody::WorldPointEvaluator<double> right_heel_evaluator;
  multibody::DistanceEvaluator<double> left_loop;
  multibody::DistanceEvaluator<double> right_loop;
  std::unordered_map<
      int, std::vector<std::pair<const Eigen::Vector3d,
                                 const drake::multibody::Frame<double>&>>>
      feet_contact_points;
  std::vector<int> fsm_states;
  std::vector<double> state_durations;
  std::vector<double> accumulated_state_durations;
  std::unique_ptr<drake::systems::Context<double>> plant_context;

  multibody::KinematicEvaluatorSet<double> evaluators;

  multibody::FixedJointEvaluator<double> left_fixed_knee_spring;
  multibody::FixedJointEvaluator<double> right_fixed_knee_spring;
  multibody::FixedJointEvaluator<double> left_fixed_ankle_spring;
  multibody::FixedJointEvaluator<double> right_fixed_ankle_spring;

  std::unique_ptr<TransTaskSpaceTrackingData> pelvis_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> stance_foot_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> left_foot_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> right_foot_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> left_foot_yz_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> right_foot_yz_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> left_hip_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> right_hip_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> left_hip_yz_tracking_data;
  std::unique_ptr<TransTaskSpaceTrackingData> right_hip_yz_tracking_data;
  std::unique_ptr<RelativeTranslationTrackingData> left_foot_rel_tracking_data;
  std::unique_ptr<RelativeTranslationTrackingData> right_foot_rel_tracking_data;
  std::unique_ptr<RelativeTranslationTrackingData> left_foot_yz_rel_tracking_data;
  std::unique_ptr<RelativeTranslationTrackingData> right_foot_yz_rel_tracking_data;
  std::unique_ptr<RelativeTranslationTrackingData> pelvis_trans_rel_tracking_data;
  std::unique_ptr<RotTaskSpaceTrackingData> pelvis_rot_tracking_data;
  std::unique_ptr<JointSpaceTrackingData> left_toe_angle_tracking_data;
  std::unique_ptr<JointSpaceTrackingData> right_toe_angle_tracking_data;
  std::unique_ptr<JointSpaceTrackingData> left_hip_yaw_tracking_data;
  std::unique_ptr<JointSpaceTrackingData> right_hip_yaw_tracking_data;

  OSCGains osc_gains_;
  OSCRunningGains osc_running_gains_;

  const int state_input_port_index_ = 0;
  const int cassie_out_input_port_index_ = 1;
  const int control_output_port_index_ = 0;
  const int controller_failure_port_index_ = 1;

  const std::string control_channel_name_ = "OSC_RUNNING";
};

}  // namespace controllers
}  // namespace examples
}  // namespace dairlib
