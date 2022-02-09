#include "trajectory_playback.h"


using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::systems {

TrajectoryPlayback::TrajectoryPlayback(const PiecewisePolynomial<double>& traj,
                                       int num_inputs,
                                       double time_offset)
    : traj_(traj) {
  this->set_name("trajectory_playback");
  cmd_output_port_ = this->DeclareVectorOutputPort("u",
      TimestampedVector<double>(num_inputs),
      &TrajectoryPlayback::CalcEffort).get_index();

  // Shift trajectory by time_offset
  traj_.shiftRight(time_offset);
}

void TrajectoryPlayback::CalcEffort(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* control) const {
  // Read in current state
  control->set_timestamp(context.get_time());
  control->SetDataVector(traj_.value(context.get_time()));
//  std::cout << "input: " << traj_.value(context.get_time()) << std::endl;
}

}  // namespace dairlib::systems
