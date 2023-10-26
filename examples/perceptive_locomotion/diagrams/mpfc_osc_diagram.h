#pragma once
#include <iostream>

// Cassie and multibody
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/osc/high_level_command.h"
#include "examples/Cassie/osc/hip_yaw_traj_gen.h"
#include "examples/Cassie/osc/heading_traj_generator.h"
#include "examples/Cassie/osc/pelvis_pitch_traj_generator.h"
#include "examples/Cassie/osc/osc_walking_gains_alip.h"
#include "examples/Cassie/osc/swing_toe_traj_generator.h"
#include "examples/Cassie/systems/cassie_out_to_radio.h"
#include "multibody/kinematic/fixed_joint_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/multibody_utils.h"

// OSC
#include "systems/controllers/osc/com_tracking_data.h"
#include "systems/controllers/osc/joint_space_tracking_data.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/options_tracking_data.h"
#include "systems/controllers/osc/relative_translation_tracking_data.h"
#include "systems/controllers/osc/rot_space_tracking_data.h"
#include "systems/controllers/osc/trans_space_tracking_data.h"

// misc
#include "systems/controllers/footstep_planning/alip_mpc_interface_system.h"
#include "systems/system_utils.h"

// drake
#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_scope_system.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"


namespace dairlib::perceptive_locomotion {

using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;
using std::pair;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::systems::InputPort;
using drake::systems::OutputPort;
using drake::trajectories::PiecewisePolynomial;


using systems::controllers::ComTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::controllers::RelativeTranslationTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;

using multibody::FixedJointEvaluator;
using multibody::WorldYawViewFrame;
using multibody::DistanceEvaluator;
using multibody::WorldPointEvaluator;
using multibody::KinematicEvaluatorSet;
using multibody::MakeNameToVelocitiesMap;
using multibody::MakeNameToPositionsMap;

class MpfcOscDiagram : public drake::systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpfcOscDiagram)

  // TODO (@Brian-Acosta) Add factory method for getting the plant
  MpfcOscDiagram(drake::multibody::MultibodyPlant<double>& plant,
                 const string& osc_gains_filename,
                 const string& mpc_gains_filename,
                 const string& osqp_settings_filename);

  const InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }
  const InputPort<double>& get_input_port_footstep_command() const {
    return get_input_port(input_port_footstep_command_);
  }
  const InputPort<double>& get_input_port_radio() const {
    return get_input_port(input_port_radio_);
  }
  const OutputPort<double>& get_output_port_actuation() const {
    return get_output_port(output_port_u_cmd_);
  }
  const OutputPort<double>& get_output_port_fsm() const {
    return get_output_port(output_port_fsm_);
  }
  const OutputPort<double>& get_output_port_alip() const {
    return get_output_port(output_port_alip_);
  }
  const OutputPort<double>& get_output_port_switching_time() const {
    return get_output_port(output_port_switching_time_);
  }
  drake::multibody::MultibodyPlant<double>& get_plant() {
    return *plant_;
  }
  void SetSwingFootPositionAtLiftoff(
      drake::systems::Context<double>* context,
      const Eigen::Vector3d& init_swing_pos) const;

 private:

  drake::multibody::MultibodyPlant<double>* plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context;

  std::pair<const Vector3d, const Frame<double>&> left_toe;
  std::pair<const Vector3d, const Frame<double>&> left_heel;
  std::pair<const Vector3d, const Frame<double>&> right_toe;
  std::pair<const Vector3d, const Frame<double>&> right_heel;
  std::pair<const Vector3d, const Frame<double>&> left_toe_mid;
  std::pair<const Vector3d, const Frame<double>&> right_toe_mid;
  std::pair<const Vector3d, const Frame<double>&> left_toe_origin;
  std::pair<const Vector3d, const Frame<double>&> right_toe_origin;


  // Create finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;
  double left_support_duration;
  double right_support_duration;
  double double_support_duration;

  vector<int> fsm_states;
  vector<double> state_durations;
  vector<int> single_support_states;
  vector<int> double_support_states;
  vector<int> unordered_fsm_states;
  vector<pair<const Vector3d, const Frame<double>&>> contact_points_in_each_state;

  vector<int> left_right_support_fsm_states;
  vector<double> left_right_support_state_durations;
  vector<pair<const Vector3d, const Frame<double>&>> left_right_foot;

  systems::controllers::SwingFootInterfaceSystemParams swing_params;
  systems::controllers::ComTrajInterfaceParams com_params;

  // Swing toe joint trajectory
  map<string, int> pos_map;
  map<string, int> vel_map;
  vector<pair<const Vector3d, const Frame<double>&>> left_foot_points;
  vector<pair<const Vector3d, const Frame<double>&>> right_foot_points;


  // Constraints in OSC
  KinematicEvaluatorSet<double> evaluators;
  DistanceEvaluator<double> left_loop;
  DistanceEvaluator<double> right_loop;
  FixedJointEvaluator<double> left_fixed_knee_spring;
  FixedJointEvaluator<double> right_fixed_knee_spring;
  FixedJointEvaluator<double> left_fixed_ankle_spring;
  FixedJointEvaluator<double> right_fixed_ankle_spring;

  WorldYawViewFrame<double> view_frame;
  WorldPointEvaluator<double> left_toe_evaluator;
  WorldPointEvaluator<double> left_heel_evaluator;
  WorldPointEvaluator<double> right_toe_evaluator;
  WorldPointEvaluator<double> right_heel_evaluator;

  // gain multipliers
  std::shared_ptr<PiecewisePolynomial<double>> swing_ft_gain_multiplier_gain_multiplier;
  std::shared_ptr<PiecewisePolynomial<double>> swing_ft_accel_gain_multiplier_gain_multiplier;

  // view frame
  std::shared_ptr<WorldYawViewFrame<double>> pelvis_view_frame;

  // Tracking data
  std::unique_ptr<TransTaskSpaceTrackingData> stance_foot_data;
  std::unique_ptr<TransTaskSpaceTrackingData> swing_foot_data;
  std::unique_ptr<RelativeTranslationTrackingData> swing_ft_data_local;
  std::unique_ptr<ComTrackingData> center_of_mass_data;
  std::unique_ptr<RotTaskSpaceTrackingData> pelvis_balance_data;
  std::unique_ptr<RotTaskSpaceTrackingData> pelvis_heading_data;
  std::unique_ptr<JointSpaceTrackingData> swing_hip_yaw_data;
  std::unique_ptr<JointSpaceTrackingData> swing_toe_data_left;
  std::unique_ptr<JointSpaceTrackingData> swing_toe_data_right;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_footstep_command_;
  drake::systems::InputPortIndex input_port_radio_;

  drake::systems::OutputPortIndex output_port_u_cmd_;
  drake::systems::OutputPortIndex output_port_fsm_;
  drake::systems::OutputPortIndex output_port_switching_time_;
  drake::systems::OutputPortIndex output_port_alip_;
};

}
