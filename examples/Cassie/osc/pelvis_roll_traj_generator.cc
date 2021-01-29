#include "pelvis_roll_traj_generator.h"

#include "multibody/multibody_utils.h"

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib::cassie::osc {

PelvisRollTrajGenerator::PelvisRollTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, const int hip_idx,
    const std::string& traj_name)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      hip_idx_(hip_idx) {
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->DeclareAbstractOutputPort("pelvis_roll_traj", traj_inst,
                                  &PelvisRollTrajGenerator::CalcTraj);
}  // namespace dairlib::cassie::osc

void PelvisRollTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  // Read in current state

  // Read in current state
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  Vector3d left_side;
  Vector3d right_side;
  left_side << 0, 1, 0;
  right_side << 0, -1, 0;
  Vector3d left_pt;
  Vector3d right_pt;
  plant_.CalcPointsPositions(*context_,
                             plant_.GetBodyByName("pelvis").body_frame(),
                             left_side, world_, &left_pt);
  plant_.CalcPointsPositions(*context_,
                             plant_.GetBodyByName("pelvis").body_frame(),
                             right_side, world_, &right_pt);
  Vector3d diff = left_pt - right_pt;
  VectorXd desired_hip_roll = VectorXd::Zero(1);
  desired_hip_roll << hip_idx_ * (atan2(diff(2), diff.head(2).norm()));

  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = PiecewisePolynomial<double>(desired_hip_roll);
}

}  // namespace dairlib::cassie::osc
