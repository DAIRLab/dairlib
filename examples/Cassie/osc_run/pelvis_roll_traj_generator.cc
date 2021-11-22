#include "examples/Cassie/osc_run/pelvis_roll_traj_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using std::string;
using std::vector;

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc {

PelvisRollTrajGenerator::PelvisRollTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::trajectories::PiecewisePolynomial<double>& hip_roll_traj,
    drake::trajectories::PiecewisePolynomial<double>& pelvis_roll_traj,
    int axis, const std::string& system_name)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      hip_roll_traj_(hip_roll_traj),
      pelvis_roll_traj_(pelvis_roll_traj),
      axis_(axis) {
  this->set_name(system_name);
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x", OutputVector<double>(plant_.num_positions(),
                                                  plant_.num_velocities(),
                                                  plant_.num_actuators()))
                    .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  clock_port_ = this->DeclareVectorInputPort("t_clock", BasicVector<double>(1))
                    .get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("pelvis_rot_traj_" + std::to_string(axis),
                                  traj_inst,
                                  &PelvisRollTrajGenerator::CalcTraj);

  DeclarePerStepDiscreteUpdateEvent(
      &PelvisRollTrajGenerator::DiscreteVariableUpdate);
}

EventStatus PelvisRollTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> PelvisRollTrajGenerator::GeneratePelvisTraj(
    const systems::OutputVector<double>* robot_output, double t,
    int fsm_state) const {
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  VectorXd correction = VectorXd::Zero(1);

  drake::math::RotationMatrix pelvis_rot =
      plant_.EvalBodyPoseInWorld(*context_, plant_.GetBodyByName("pelvis"))
          .rotation();
  drake::math::RollPitchYawd pelvis_rpy = drake::math::RollPitchYaw(pelvis_rot);
  double pelvis_roll = pelvis_rpy.roll_angle();

  correction << pelvis_roll;
  std::vector<double> breaks = hip_roll_traj_.get_segment_times();
  VectorXd breaks_vector = Map<VectorXd>(breaks.data(), breaks.size());
  MatrixXd offset_angles = correction.replicate(1, breaks.size());
//  for (int i = 0; i < breaks_vector.size(); ++i){
//    offset_angles.col(i) = i * offset_angles.col(i) / breaks_vector.size();
//  }
  PiecewisePolynomial<double> offset_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(breaks_vector, offset_angles);
  return hip_roll_traj_ + offset_traj;
}

void PelvisRollTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  // Read in finite state machine
  const auto& fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  const auto& clock =
      this->EvalVectorInput(context, clock_port_)->get_value()(0);

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (fsm_state == 0 || fsm_state == 1) {
    *casted_traj = GeneratePelvisTraj(robot_output, clock, fsm_state);
  }
}

}  // namespace dairlib::examples::osc