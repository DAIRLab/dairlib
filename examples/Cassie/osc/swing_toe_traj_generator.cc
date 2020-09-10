#include "swing_toe_traj_generator.h"

#include "multibody/multibody_utils.h"

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib::cassie::osc {

SwingToeTrajGenerator::SwingToeTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, int swing_toe_idx,
    const std::vector<std::pair<const Eigen::Vector3d,
                                const drake::multibody::Frame<double>&>>&
        feet_contact_points,
    const std::string& traj_name)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      swing_toe_idx_(swing_toe_idx),
      feet_contact_points_(feet_contact_points) {
  DRAKE_DEMAND(feet_contact_points_.size() == 2);
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->DeclareAbstractOutputPort("swing_toe_traj", traj_inst,
                                  &SwingToeTrajGenerator::CalcTraj);
}

void SwingToeTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  // Read in current state

  // Read in current state
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  double swing_toe_angle = q[swing_toe_idx_];

  Vector3d pt_0;
  Vector3d pt_1;
  plant_.CalcPointsPositions(*context_, feet_contact_points_[0].second,
                             feet_contact_points_[0].first, world_, &pt_0);
  plant_.CalcPointsPositions(*context_, feet_contact_points_[1].second,
                             feet_contact_points_[1].first, world_, &pt_1);

  VectorXd foot = pt_0 - pt_1;
  double deviation_from_ground_plane = (atan2(foot(2), foot.head(2).norm()));
  // Get current difference between
  VectorXd des_swing_toe_angle = VectorXd(1);
  des_swing_toe_angle << swing_toe_angle + deviation_from_ground_plane;

  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = PiecewisePolynomial<double>(des_swing_toe_angle);
}

}  // namespace dairlib::cassie::osc
