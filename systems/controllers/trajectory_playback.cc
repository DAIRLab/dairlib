#include "trajectory_playback.h"


using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::systems {

TrajectoryPlayback::TrajectoryPlayback(const PiecewisePolynomial<double>& traj,
                                       int num_inputs,
                                       double time_offset)
    : traj_(traj) {
  this->set_name("trajectory_playback");
  this->DeclareVectorOutputPort(
      BasicVector<double>(num_inputs),
      &TrajectoryPlayback::CalcEffort);

  // Shift trajectory by time_offset
  traj_.shiftRight(time_offset);
}

void TrajectoryPlayback::CalcEffort(
    const drake::systems::Context<double>& context,
    BasicVector<double>* control) const {
  // Read in current state
  control->SetFromVector(traj_.value(context.get_time()));
}

}  // namespace dairlib::systems
