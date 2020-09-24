#include "examples/Cassie/osc/high_level_command.h"

#include <math.h>

#include <string>

#include "multibody/multibody_utils.h"

#include "drake/math/quaternion.h"

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
    drake::systems::Context<double>* context,
    const Vector2d& global_target_position,
    const Vector2d& params_of_no_turning)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")),
      global_target_position_(global_target_position),
      params_of_no_turning_(params_of_no_turning) {
  // Input/Output Setup
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

  // Discrete state which stores previous timestamp
  prev_time_idx_ = DeclareDiscreteState(VectorXd::Zero(1));

  // Discrete state which stores the desired yaw velocity
  des_yaw_vel_idx_ = DeclareDiscreteState(VectorXd::Zero(1));

  // Discrete state which stores the desired horizontal velocity
  des_horizontal_vel_idx_ = DeclareDiscreteState(VectorXd::Zero(2));
}

EventStatus HighLevelCommand::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  double current_time = context.get_time();
  auto prev_time =
      discrete_state->get_mutable_vector(prev_time_idx_).get_mutable_value();

  if (current_time > prev_time(0)) {
    prev_time(0) = current_time;

    // Read in current state
    const OutputVector<double>* robotOutput =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
    VectorXd q = robotOutput->GetPositions();
    VectorXd v = robotOutput->GetVelocities();

    multibody::SetPositionsIfNew<double>(plant_, q, context_);

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
    des_yaw_vel = std::min(vel_max_yaw_, std::max(vel_min_yaw_, des_yaw_vel));

    // Convex combination of 0 and desired yaw velocity
    double weight = 1 / (1 + exp(-params_of_no_turning_(0) *
                                 (global_com_pos_to_target_pos.norm() -
                                  params_of_no_turning_(1))));
    double desired_filtered_yaw_vel = (1 - weight) * 0 + weight * des_yaw_vel;
    discrete_state->get_mutable_vector(des_yaw_vel_idx_).get_mutable_value()
        << desired_filtered_yaw_vel;

    //////////// Get desired horizontal vel ////////////
    // Calculate the current-desired yaw angle difference
    double desired_filtered_yaw =
        (1 - weight) * approx_pelvis_yaw + weight * desired_yaw;
    double heading_error = desired_filtered_yaw - approx_pelvis_yaw;

    // Apply walking speed control only when the robot is facing the target
    // position.
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
      Vector3d local_com_pos_to_target_pos = drake::math::quatRotateVec(
          quad_conj, global_com_pos_to_target_pos_3d);
      Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

      // Sagital plane position PD control
      double com_vel_sagital = local_com_vel(0);
      double des_sagital_vel =
          kp_pos_sagital_ *
              (local_com_pos_to_target_pos(0) + target_pos_offset_) +
          kd_pos_sagital_ * (-com_vel_sagital);
      des_sagital_vel = std::min(vel_max_sagital_,
                                 std::max(vel_min_sagital_, des_sagital_vel));

      // Frontal plane position PD control.  TODO(yminchen): tune this
      double com_vel_lateral = local_com_vel(1);
      double des_lateral_vel =
          kp_pos_lateral_ * (local_com_pos_to_target_pos(1)) +
          kd_pos_lateral_ * (-com_vel_lateral);
      des_lateral_vel = std::min(vel_max_lateral_,
                                 std::max(vel_min_lateral_, des_lateral_vel));

      discrete_state->get_mutable_vector(des_horizontal_vel_idx_)
              .get_mutable_value()
          << des_sagital_vel,
          des_lateral_vel;
    } else {
      discrete_state->get_mutable_vector(des_horizontal_vel_idx_)
              .get_mutable_value()
          << 0,
          0;
    }
  }

  return EventStatus::Succeeded();
}

void HighLevelCommand::CopyHeadingAngle(const Context<double>& context,
                                        BasicVector<double>* output) const {
  double desired_heading_pos =
      context.get_discrete_state(des_yaw_vel_idx_).get_value()(0);

  // Assign
  output->get_mutable_value() << desired_heading_pos;
}

void HighLevelCommand::CopyDesiredHorizontalVel(
    const Context<double>& context, BasicVector<double>* output) const {
  auto delta_CP_3D_global =
      context.get_discrete_state(des_horizontal_vel_idx_).get_value();

  // Assign
  output->get_mutable_value() = delta_CP_3D_global;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
