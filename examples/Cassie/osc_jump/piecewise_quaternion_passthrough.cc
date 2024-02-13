#include "piecewise_quaternion_passthrough.h"

#include "multibody/multibody_utils.h"

using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc_jump {

QuaternionTrajectoryPassthrough::QuaternionTrajectoryPassthrough(
    const PiecewiseQuaternionSlerp<double>& traj, const std::string& traj_name,
    double time_offset)
    : traj_(traj) {
  PiecewiseQuaternionSlerp<double> empty_pp_traj;
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->set_name(traj_name);
  this->DeclareAbstractOutputPort(traj_name, traj_inst,
                                  &QuaternionTrajectoryPassthrough::CalcTraj);

  // Shift trajectory by time_offset
}

void QuaternionTrajectoryPassthrough::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  // Read in current state
  auto* casted_traj =
  (PiecewiseQuaternionSlerp<double>*)dynamic_cast<PiecewiseQuaternionSlerp<double>*>(
      traj);
  *casted_traj = traj_;
}

}  // namespace dairlib::examples::osc_jump
