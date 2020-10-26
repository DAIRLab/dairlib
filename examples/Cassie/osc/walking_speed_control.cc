#include "examples/Cassie/osc/walking_speed_control.h"

#include <math.h>
#include <string>

#include "multibody/multibody_utils.h"

#include "drake/math/quaternion.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;

using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace cassie {
namespace osc {

WalkingSpeedControl::WalkingSpeedControl(
    const drake::multibody::MultibodyPlant<double>& plant,
    Context<double>* context, int footstep_option, double swing_phase_duration)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")),
      swing_phase_duration_(swing_phase_duration),
      is_using_predicted_com_(swing_phase_duration > 0) {
  DRAKE_DEMAND(0 <= footstep_option && footstep_option <= 1);

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  xy_port_ = this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(2),
                                &WalkingSpeedControl::CalcFootPlacement);

  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  if (is_using_predicted_com_) {
    fsm_switch_time_port_ =
        this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

    com_port_ =
        this->DeclareAbstractInputPort(
                "com_traj",
                drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
  }

  // TODO(yminchen): Gains are not tuned yet. Do we need two sets of gains for
  //  moving forward and backward?
  // Control gains
  if (footstep_option == 0) {
    // For Capture point
    k_fp_ff_sagital_ = 0.16;
    k_fp_fb_sagital_ = 0.04;
    k_fp_ff_lateral_ = 0.08;
    k_fp_fb_lateral_ = 0.02;
  } else if (footstep_option == 1) {
    // For LIPM neutral point
    k_fp_ff_sagital_ = 0;
    k_fp_fb_sagital_ = 0.06;
    k_fp_ff_lateral_ = 0;
    k_fp_fb_lateral_ = 0.12;
  }
}

void WalkingSpeedControl::CalcFootPlacement(const Context<double>& context,
                                            BasicVector<double>* output) const {
  // Read in finite state machine
  const BasicVector<double>* des_hor_vel_output =
      (BasicVector<double>*)this->EvalVectorInput(context, xy_port_);
  VectorXd des_hor_vel = des_hor_vel_output->get_value();

  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();

  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  Vector3d com_vel;
  if (is_using_predicted_com_) {
    // Get the predicted center of mass velocity
    VectorXd prev_lift_off_time =
        this->EvalVectorInput(context, fsm_switch_time_port_)->get_value();
    double end_time_of_swing_phase =
        prev_lift_off_time(0) + swing_phase_duration_;

    const drake::AbstractValue* com_traj_output =
        this->EvalAbstractInput(context, com_port_);
    DRAKE_ASSERT(com_traj_output != nullptr);
    const auto& com_traj =
        com_traj_output->get_value<drake::trajectories::Trajectory<double>>();
    com_vel = com_traj.MakeDerivative(1)->value(end_time_of_swing_phase);
  } else {
    // Get the current center of mass velocity
    MatrixXd J(3, plant_.num_velocities());
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, world_, world_, &J);
    com_vel = J * v;
  }

  // Filter the com vel
  if (robot_output->get_timestamp() != last_timestamp_) {
    double dt = robot_output->get_timestamp() - last_timestamp_;
    last_timestamp_ = robot_output->get_timestamp();
    double alpha =
        2 * M_PI * dt * cutoff_freq_ / (2 * M_PI * dt * cutoff_freq_ + 1);
    filterred_com_vel_ = alpha * com_vel + (1 - alpha) * filterred_com_vel_;
  }
  com_vel = filterred_com_vel_;

  // Extract quaternion from floating base position
  Quaterniond Quat(q(0), q(1), q(2), q(3));
  Quaterniond Quat_conj = Quat.conjugate();
  Vector4d quat(q.head(4));
  Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(), Quat_conj.y(),
                     Quat_conj.z());

  // Calculate local velocity
  Vector3d local_com_vel =
      drake::math::quatRotateVec(quad_conj, com_vel);

  //////////////////// Sagital ////////////////////
  Vector3d delta_x_fs_sagital_3D_global(0, 0, 0);

  // Position Control
  double com_vel_sagital = local_com_vel(0);
  double des_sagital_vel = des_hor_vel(0);

  // Velocity control
  double delta_x_fs_sagital =
      -k_fp_ff_sagital_ * des_sagital_vel -
      k_fp_fb_sagital_ * (des_sagital_vel - com_vel_sagital);
  Vector3d delta_x_fs_sagital_3D_local(delta_x_fs_sagital, 0, 0);
  delta_x_fs_sagital_3D_global =
      drake::math::quatRotateVec(quat, delta_x_fs_sagital_3D_local);

  //////////////////// Lateral ////////////////////  TODO(yminchen): tune this
  Vector3d delta_x_fs_lateral_3D_global(0, 0, 0);

  // Position Control
  double com_vel_lateral = local_com_vel(1);
  double des_lateral_vel = des_hor_vel(1);

  // Velocity control
  double delta_x_fs_lateral =
      -k_fp_ff_lateral_ * des_lateral_vel -
      k_fp_fb_lateral_ * (des_lateral_vel - com_vel_lateral);
  Vector3d delta_x_fs_lateral_3D_local(0, delta_x_fs_lateral, 0);
  delta_x_fs_lateral_3D_global =
      drake::math::quatRotateVec(quat, delta_x_fs_lateral_3D_local);

  // Assign foot placement
  output->get_mutable_value() =
      (delta_x_fs_sagital_3D_global + delta_x_fs_lateral_3D_global).head(2);
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
