#include "planar_lipm_walking_speed_control.h"
#include <cmath>
#include "multibody/multibody_utils.h"

using std::cout;
using std::endl;

using dairlib::systems::OutputVector;

using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::trajectories::PiecewisePolynomial;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;


namespace dairlib {
namespace koopman_examples {

PlanarLipmWalkingSpeedControl::PlanarLipmWalkingSpeedControl(
    const drake::multibody::MultibodyPlant<double>& plant,
    Context<double>* context, double k_ff, double k_fb, std::string base_name,
    double swing_phase_duration) :
    plant_(plant),
    context_(context),
    world_(plant.world_frame()),
    pelvis_(plant.GetBodyByName(base_name)),
    swing_phase_duration_(swing_phase_duration),
    is_using_predicted_com_(swing_phase_duration > 0),
    k_ff_(k_ff), k_fb_(k_fb) {

  state_port_ =
      this->DeclareVectorInputPort("x, u, t", OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators()))
          .get_index();

  v_des_port_ = this->DeclareVectorInputPort("v_des",BasicVector<double>(1)).get_index();
  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  if (is_using_predicted_com_) {
    fsm_switch_time_port_ =
        this->DeclareVectorInputPort("fsm_switch",BasicVector<double>(1)).get_index();

    com_port_ =
        this->DeclareAbstractInputPort(
                "com_traj",
                drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
  }
  this->DeclareVectorOutputPort("out", BasicVector<double>(2),
                                &PlanarLipmWalkingSpeedControl::CalcFootPlacement);
}

void PlanarLipmWalkingSpeedControl::CalcFootPlacement(const Context<double> &context,
    drake::systems::BasicVector<double> *output) const {

  VectorXd des_vel = this->EvalVectorInput(context, v_des_port_)->get_value();

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

  double delta_x = -k_ff_ * des_vel(0) -
                    k_fb_ * (des_vel(0) - com_vel(0));

  output->get_mutable_value() = Vector2d(delta_x,0);
}


}
}