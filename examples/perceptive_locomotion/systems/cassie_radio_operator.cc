#include "cassie_radio_operator.h"
#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"
#include <iostream>

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::BasicVector;

CassieRadioOperator::CassieRadioOperator(
    const drake::multibody::MultibodyPlant<double>& plant,
    Context<double>* context) : plant_(plant), context_(context) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)).get_index();

  input_port_target_xy_ = DeclareVectorInputPort(
      "target_position", 2).get_index();

  output_port_radio_ = DeclareVectorOutputPort(
      "radio_channels", 18, &CassieRadioOperator::CopyRadio).get_index();

  output_port_horizontal_velocity_ = DeclareVectorOutputPort(
      "vel_xy", 2, &CassieRadioOperator::CopyVdes).get_index();

  vdes_state_index_ = DeclareDiscreteState(3);

  DeclarePerStepDiscreteUpdateEvent(&CassieRadioOperator::DiscreteUpdate);
}

void CassieRadioOperator::CopyVdes(const Context<double> &context,
                                   BasicVector<double> *vdes_xy) const {
  Eigen::Vector2d xy =
      context.get_discrete_state(vdes_state_index_).get_value().tail<2>();
  vdes_xy->SetFromVector(xy);
}

void CassieRadioOperator::CopyRadio(const Context<double> &context,
                                    BasicVector<double> * radio) const {
  Eigen::Vector2d vdes =
      context.get_discrete_state(vdes_state_index_).get_value();
  Eigen::VectorXd raw_radio = Eigen::VectorXd::Zero(kRadioDim);
  raw_radio(kRotChannel) = rot_vel_to_radio * vdes(0);
  raw_radio(kXChannel) = x_vel_to_radio * vdes(1);
  raw_radio(kYChannel) = y_vel_to_radio * vdes(2);

  radio->set_value(raw_radio);
}

drake::systems::EventStatus CassieRadioOperator::DiscreteUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double> *discrete_values) const {

  Eigen::Vector2d target_xy =
      EvalVectorInput(context, input_port_target_xy_)->get_value();
  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_));
  Eigen::VectorXd q = robot_output->GetPositions();
  Eigen::VectorXd v = robot_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);


  drake::math::RigidTransformd pose =
      plant_.GetBodyByName("pelvis").EvalPoseInWorld(*context_);

  Eigen::Vector3d pos = pose.translation();
  Eigen::Vector3d heading = pose.rotation().matrix().col(0);

  Eigen::Vector2d to_target = target_xy - pos.head<2>();

  if (to_target.squaredNorm() > 1) {
    to_target.normalize();
  }

  double heading_error = atan2(heading(1), heading(0));
  double yaw_rate = std::clamp(heading_error, -rot_vel_to_radio, rot_vel_to_radio);

  double ramp_factor = std::min(robot_output->get_timestamp(), 1.0);

  Eigen::Vector2d vdes = ramp_factor * 0.5 * pose.rotation().matrix().topLeftCorner<2,2>().transpose() * to_target;
  vdes -= 0.5 * pose.rotation().matrix().topLeftCorner<2,2>().transpose() * v.segment<2>(3);


  discrete_values->get_mutable_value(vdes_state_index_) << yaw_rate, vdes(0), vdes(1);

  return drake::systems::EventStatus::Succeeded();

}

}
}