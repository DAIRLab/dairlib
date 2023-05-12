#include "pelvis_pitch_traj_generator.h"
#include <math.h>
#include <string>
#include "multibody/multibody_utils.h"

using std::string;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;

using drake::trajectories::PiecewisePolynomial;
using drake::math::RotationMatrixd;
using drake::math::RollPitchYawd;

namespace dairlib {
namespace cassie {
namespace osc {

PelvisPitchTrajGenerator::PelvisPitchTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.num_positions(),
                                          plant.num_velocities(),
                                          plant.num_actuators()))
      .get_index();
  des_pitch_port_ =
      this->DeclareVectorInputPort("pelvis_pitch", 1).get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("pelvis_quat", traj_inst,
                                  &PelvisPitchTrajGenerator::CalcPitchTraj);

  prev_timestamp_idx_ = DeclareDiscreteState(1);
  filtered_pitch_idx_ = DeclareDiscreteState(1);

  DeclareForcedUnrestrictedUpdateEvent(&PelvisPitchTrajGenerator::PitchFilterUpdate);
}

drake::systems::EventStatus PelvisPitchTrajGenerator::PitchFilterUpdate(
    const Context<double> &context, drake::systems::State<double> *state) const {
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));
  double t = robot_output->get_timestamp();
  double t_prev = state->get_discrete_state(prev_timestamp_idx_).get_value()(0);
  double prev_pitch = state->get_discrete_state(filtered_pitch_idx_).get_value()(0);
  double des_pitch = EvalVectorInput(context, des_pitch_port_)->get_value()(0);
  double dt = t - t_prev;
  if (dt < 0.1) {
    double tau = .800; // 200  ms time constant
    double alpha = dt / (tau + dt);
    des_pitch = alpha * des_pitch + (1.0 - alpha) * prev_pitch;
  }
  state->get_mutable_discrete_state(prev_timestamp_idx_).set_value(drake::Vector1d::Constant(t));
  state->get_mutable_discrete_state(filtered_pitch_idx_).set_value(drake::Vector1d::Constant(des_pitch));
  return drake::systems::EventStatus::Succeeded();
}

void PelvisPitchTrajGenerator::CalcPitchTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {

  double des_pitch = context.get_discrete_state(filtered_pitch_idx_).get_value()(0);
  // Read in current state
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, state_port_));
  multibody::SetPositionsIfNew<double>(
      plant_, robot_output->GetPositions(), context_);

  const RotationMatrixd& R_WP =
      plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation();
  RollPitchYawd rpy(R_WP);
  rpy.set_roll_angle(0);
  rpy.set_pitch_angle(-des_pitch);

  Quaterniond quat = rpy.ToQuaternion();
  Eigen::Vector4d vec;
  vec << quat.w(), quat.vec();
  const auto pp = PiecewisePolynomial<double>(vec);

  // Assign traj
  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = pp;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
