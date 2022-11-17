#include "lcm_trajectory_systems.h"

#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib {
namespace systems {

using drake::trajectories::PiecewisePolynomial;

LcmTrajectoryReceiver::LcmTrajectoryReceiver(std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewisePolynomial<double> traj_inst(Eigen::VectorXd(0));
  this->set_name(trajectory_name_);
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort(trajectory_name_, traj_inst,
                                      &LcmTrajectoryReceiver::OutputTrajectory)
          .get_index();
}

void LcmTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    PiecewisePolynomial<double>* output_trajectory) const {
  const auto& lcm_traj =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, trajectory_input_port_);
  auto lcm_trajs = LcmTrajectory(lcm_traj->saved_traj);
  const auto trajectory_block = lcm_trajs.GetTrajectory(trajectory_name_);

  *output_trajectory = PiecewisePolynomial<double>::FirstOrderHold(
      trajectory_block.time_vector, trajectory_block.datapoints);
}

}  // namespace systems
}  // namespace dairlib
