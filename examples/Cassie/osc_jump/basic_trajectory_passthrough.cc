#include "basic_trajectory_passthrough.h"

#include "multibody/multibody_utils.h"

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc_jump {

BasicTrajectoryPassthrough::BasicTrajectoryPassthrough(
    const PiecewisePolynomial<double>& traj, const std::string& traj_name,
    double time_offset)
    : traj_(traj) {
  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->set_name(traj_name);
  this->DeclareAbstractOutputPort(traj_name, traj_inst,
                                  &BasicTrajectoryPassthrough::CalcTraj);

  // Shift trajectory by time_offset
  traj_.shiftRight(time_offset);
}

void BasicTrajectoryPassthrough::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  // Read in current state
  auto* casted_traj =
  (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
      traj);
  *casted_traj = traj_;
}

}  // namespace dairlib::examples::osc_jump
