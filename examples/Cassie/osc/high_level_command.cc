#include "examples/Cassie/osc/high_level_command.h"

#include <math.h>

#include <string>

#include "dairlib/lcmt_cassie_out.hpp"
#include "multibody/multibody_utils.h"

#include "drake/math/quaternion.h"
#include "drake/math/saturate.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Quaterniond;

using dairlib::systems::OutputVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;

using drake::multibody::JacobianWrtVariable;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace cassie {
namespace osc {

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double vel_scale_rot,
    double vel_scale_trans_sagital, double vel_scale_trans_lateral)
    : HighLevelCommand(plant, context) {
  cassie_out_port_ =
      this->DeclareAbstractInputPort("lcmt_cassie_output",
                                     drake::Value<dairlib::lcmt_cassie_out>{})
          .get_index();
  use_radio_command_ = true;

  vel_scale_rot_ = vel_scale_rot;
  vel_scale_trans_sagital_ = vel_scale_trans_sagital;
  vel_scale_trans_lateral_ = vel_scale_trans_lateral;
}

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double kp_yaw, double kd_yaw,
    double vel_max_yaw, double kp_pos_sagital, double kd_pos_sagital,
    double vel_max_sagital, double kp_pos_lateral, double kd_pos_lateral,
    double vel_max_lateral, double target_pos_offset,
    const Vector2d& global_target_position,
    const Vector2d& params_of_no_turning)
    : HighLevelCommand(plant, context) {
  use_radio_command_ = false;
  kp_yaw_ = kp_yaw;
  kd_yaw_ = kd_yaw;
  vel_max_yaw_ = vel_max_yaw;
  kp_pos_sagital_ = kp_pos_sagital;
  kd_pos_sagital_ = kd_pos_sagital;
  vel_max_sagital_ = vel_max_sagital;
  target_pos_offset_ = target_pos_offset;
  kp_pos_lateral_ = kp_pos_lateral;
  kd_pos_lateral_ = kd_pos_lateral;
  vel_max_lateral_ = vel_max_lateral;

  global_target_position_ = global_target_position;
  params_of_no_turning_ = params_of_no_turning;
}

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")) {
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();

  yaw_port_ = this->DeclareVectorOutputPort(BasicVector<double>(1),
                                            &HighLevelCommand::CopyHeadingAngle)
                  .get_index();
  xy_port_ =
      this->DeclareVectorOutputPort(BasicVector<double>(2),
                                    &HighLevelCommand::CopyDesiredHorizontalVel)
          .get_index();
  // Declare update event
  DeclarePerStepDiscreteUpdateEvent(&HighLevelCommand::DiscreteVariableUpdate);

  // Discrete state which stores the desired yaw velocity
  des_vel_idx_ = DeclareDiscreteState(VectorXd::Zero(3));
}

EventStatus HighLevelCommand::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (use_radio_command_) {
    const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
        context, cassie_out_port_);
    // TODO(yangwill) make sure there is a message available
    // des_vel indices: 0: yaw_vel (right joystick left/right)
    //                  1: saggital_vel (left joystick up/down)
    //                  2: lateral_vel (left joystick left/right)
    Vector3d des_vel;
    des_vel << vel_scale_rot_ * cassie_out->pelvis.radio.channel[3],
        vel_scale_trans_sagital_ * cassie_out->pelvis.radio.channel[0],
        vel_scale_trans_lateral_ * cassie_out->pelvis.radio.channel[1];
    discrete_state->get_mutable_vector(des_vel_idx_).set_value(des_vel);
  } else {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(CalcCommandFromTargetPosition(context));
  }

  return EventStatus::Succeeded();
}

VectorXd HighLevelCommand::CalcCommandFromTargetPosition(
    const Context<double>& context) const {
  // Read in current state
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  VectorXd v = robotOutput->GetVelocities();

  plant_.SetPositions(context_, q);

  // Get center of mass position and velocity
  Vector3d com_pos = plant_.CalcCenterOfMassPosition(*context_);
  MatrixXd J(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, world_, world_, &J);
  Vector3d com_vel = J * v;

  //////////// Get desired yaw velocity ////////////
  // Get approximated heading angle of pelvis
  Vector3d pelvis_heading_vec =
      plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desired heading angle of pelvis
  Vector2d global_com_pos_to_target_pos =
      global_target_position_ - com_pos.segment(0, 2);
  double desired_yaw =
      atan2(global_com_pos_to_target_pos(1), global_com_pos_to_target_pos(0));

  // Get current yaw velocity
  double yaw_vel = v(2);

  // PD position control
  double des_yaw_vel =
      kp_yaw_ * (desired_yaw - approx_pelvis_yaw) + kd_yaw_ * (-yaw_vel);
  des_yaw_vel = drake::math::saturate(des_yaw_vel, -vel_max_yaw_, vel_max_yaw_);

  // Convex combination of 0 and desired yaw velocity
  double weight = 1 / (1 + exp(-params_of_no_turning_(0) *
                               (global_com_pos_to_target_pos.norm() -
                                params_of_no_turning_(1))));
  double desired_filtered_yaw_vel = (1 - weight) * 0 + weight * des_yaw_vel;

  //////////// Get desired horizontal vel ////////////
  // Calculate the current-desired yaw angle difference
  double desired_filtered_yaw =
      (1 - weight) * approx_pelvis_yaw + weight * desired_yaw;
  double heading_error = desired_filtered_yaw - approx_pelvis_yaw;

  // Apply walking speed control only when the robot is facing the target
  // position.
  double des_sagital_vel = 0;
  double des_lateral_vel = 0;
  if (heading_error < M_PI / 2 && heading_error > -M_PI / 2) {
    // Extract quaternion from floating base position
    Quaterniond Quat(q(0), q(1), q(2), q(3));
    Quaterniond Quat_conj = Quat.conjugate();
    Vector4d quat(q.head(4));
    Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(), Quat_conj.y(),
                       Quat_conj.z());

    // Calculate local target position and com velocity
    Vector3d global_com_pos_to_target_pos_3d;
    global_com_pos_to_target_pos_3d << global_com_pos_to_target_pos, 0;
    Vector3d local_com_pos_to_target_pos =
        drake::math::quatRotateVec(quad_conj, global_com_pos_to_target_pos_3d);
    Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

    // Sagital plane position PD control
    double com_vel_sagital = local_com_vel(0);
    des_sagital_vel = kp_pos_sagital_ * (local_com_pos_to_target_pos(0) +
                                         target_pos_offset_) +
                      kd_pos_sagital_ * (-com_vel_sagital);
    des_sagital_vel = drake::math::saturate(des_sagital_vel, -vel_max_sagital_,
                                            vel_max_sagital_);

    // Frontal plane position PD control.  TODO(yminchen): tune this
    double com_vel_lateral = local_com_vel(1);
    des_lateral_vel = kp_pos_lateral_ * (local_com_pos_to_target_pos(1)) +
                      kd_pos_lateral_ * (-com_vel_lateral);
    des_lateral_vel = drake::math::saturate(des_lateral_vel, -vel_max_lateral_,
                                            vel_max_lateral_);
  }
  Vector3d des_vel;
  des_vel << desired_filtered_yaw_vel, des_sagital_vel, des_lateral_vel;

  return des_vel;
}

void HighLevelCommand::CopyHeadingAngle(const Context<double>& context,
                                        BasicVector<double>* output) const {
  double desired_heading_pos =
      context.get_discrete_state(des_vel_idx_).get_value()(0);
  // Assign
  output->get_mutable_value() << desired_heading_pos;
}

void HighLevelCommand::CopyDesiredHorizontalVel(
    const Context<double>& context, BasicVector<double>* output) const {
  auto delta_CP_3D_global =
      context.get_discrete_state(des_vel_idx_).get_value().tail(2);

  // Assign
  output->get_mutable_value() = delta_CP_3D_global;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
