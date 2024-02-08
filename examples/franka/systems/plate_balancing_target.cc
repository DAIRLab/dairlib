#include "plate_balancing_target.h"

#include <iostream>

#include "dairlib/lcmt_radio_out.hpp"

using dairlib::systems::StateVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

PlateBalancingTargetGenerator::PlateBalancingTargetGenerator(
    const MultibodyPlant<double>& object_plant, double end_effector_thickness,
    double target_threshold)
    : end_effector_thickness_(end_effector_thickness),
      target_threshold_(target_threshold) {
  // Input/Output Setup
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  tray_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(object_plant.num_positions(),
                                              object_plant.num_velocities()))
          .get_index();
  end_effector_target_port_ =
      this->DeclareVectorOutputPort(
              "end_effector_target", BasicVector<double>(3),
              &PlateBalancingTargetGenerator::CalcEndEffectorTarget)
          .get_index();
  tray_target_port_ = this->DeclareVectorOutputPort(
                              "tray_target", BasicVector<double>(7),
                              &PlateBalancingTargetGenerator::CalcTrayTarget)
                          .get_index();
  sequence_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  within_target_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  time_entered_target_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  DeclareForcedDiscreteUpdateEvent(
      &PlateBalancingTargetGenerator::DiscreteVariableUpdate);
}

EventStatus PlateBalancingTargetGenerator::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const StateVector<double>* tray_state =
      (StateVector<double>*)this->EvalVectorInput(context, tray_state_port_);
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);

  // Ugly FSM
  int current_sequence = context.get_discrete_state(sequence_index_)[0];
  int within_target = context.get_discrete_state(within_target_index_)[0];
  int time_entered_target =
      context.get_discrete_state(time_entered_target_index_)[0];
  if (current_sequence == 0) {
    if ((tray_state->GetPositions().tail(3) - first_target_).norm() <
        target_threshold_) {
      if (within_target ==
          0) {  // set the time of when the tray first hits the target
        discrete_state->get_mutable_value(time_entered_target_index_)[0] =
            context.get_time();
      }
      discrete_state->get_mutable_value(within_target_index_)[0] = 1;
    }
    if (within_target == 1 &&
        (context.get_time() - time_entered_target) > 1.0) {
      discrete_state->get_mutable_value(within_target_index_)[0] = 0;
      discrete_state->get_mutable_value(sequence_index_)[0] = 1;
    }
  } else if (current_sequence == 1) {
    if ((tray_state->GetPositions().tail(3) - second_target_).norm() <
        target_threshold_) {
      if (within_target ==
          0) {  // set the time of when the tray first hits the target
        discrete_state->get_mutable_value(time_entered_target_index_)[0] =
            context.get_time();
      }
      discrete_state->get_mutable_value(within_target_index_)[0] = 1;
    }
    if (within_target == 1 &&
        (context.get_time() - time_entered_target) > delay_at_top_) {
      discrete_state->get_mutable_value(within_target_index_)[0] = 0;
      discrete_state->get_mutable_value(sequence_index_)[0] = 2;
    }
  } else if (current_sequence == 2) {
    if ((tray_state->GetPositions().tail(3) - third_target_).norm() <
        target_threshold_) {
      discrete_state->get_mutable_value(sequence_index_)[0] = 3;
    }
  }
  if (current_sequence == 3 && radio_out->channel[15] < 0) {
    discrete_state->get_mutable_value(sequence_index_)[0] = 0;
  }
  return EventStatus::Succeeded();
}

void PlateBalancingTargetGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& first_target, const Eigen::Vector3d& second_target,
    const Eigen::Vector3d& third_target, double x_scale, double y_scale,
    double z_scale) {
  first_target_ = first_target;
  second_target_ = second_target;
  third_target_ = third_target;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void PlateBalancingTargetGenerator::CalcEndEffectorTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);

  VectorXd end_effector_position = first_target_;
  // Update target if remote trigger is active
  if (context.get_discrete_state(sequence_index_)[0] == 1) {
    end_effector_position = second_target_;  // raise the tray once it is close
  } else if (context.get_discrete_state(sequence_index_)[0] == 2 ||
             context.get_discrete_state(sequence_index_)[0] == 3) {
    end_effector_position = third_target_;  // put the tray back
  }
  end_effector_position[2] -=
      end_effector_thickness_;  // place end effector below tray
  if (end_effector_position[0] > 0.6) {
    end_effector_position[0] = 0.6;  // keep it within the workspace
  }
  if (radio_out->channel[13] > 0) {
    end_effector_position(0) += radio_out->channel[0] * x_scale_;
    end_effector_position(1) += radio_out->channel[1] * y_scale_;
    end_effector_position(2) += radio_out->channel[2] * z_scale_;
  }
  target->SetFromVector(end_effector_position);
}

void PlateBalancingTargetGenerator::CalcTrayTarget(
    const drake::systems::Context<double>& context,
    BasicVector<double>* target) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  VectorXd target_tray_state = VectorXd::Zero(7);
  VectorXd tray_position = first_target_;

  if (context.get_discrete_state(sequence_index_)[0] == 1) {
    tray_position = second_target_;  // raise the tray once it is close
  } else if (context.get_discrete_state(sequence_index_)[0] == 2 ||
             context.get_discrete_state(sequence_index_)[0] == 3) {
    tray_position = third_target_;  // raise the tray once it is close
  }
  tray_position(0) += radio_out->channel[0] * x_scale_;
  tray_position(1) += radio_out->channel[1] * y_scale_;
  tray_position(2) += radio_out->channel[2] * z_scale_;
  target_tray_state << 1, 0, 0, 0, tray_position;  // tray orientation is flat
  target->SetFromVector(target_tray_state);
}

}  // namespace systems
}  // namespace dairlib