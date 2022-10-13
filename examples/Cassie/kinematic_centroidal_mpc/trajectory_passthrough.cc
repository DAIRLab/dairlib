#include "trajectory_passthrough.h"

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::systems {

TrajectoryPassthrough::TrajectoryPassthrough(
    const std::string& traj_name, int start_index, int length) : start_index_(
    start_index), length_(length) {

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "lambda_reference",
              drake::Value<PiecewisePolynomial<double>>(PiecewisePolynomial<double>()))
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->set_name(traj_name);
  this->DeclareAbstractOutputPort(traj_name, traj_inst,
                                  &TrajectoryPassthrough::CalcTraj);

}

void TrajectoryPassthrough::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  const auto& state_traj =
      this->EvalInputValue<drake::trajectories::PiecewisePolynomial<double >> (
          context, trajectory_port_);
  auto* casted_traj = (PiecewisePolynomial<double>*)
  dynamic_cast<PiecewisePolynomial<double>*>(traj);
  *casted_traj = state_traj->Block(start_index_, 0, length_, 1);
}

}  // namespace dairlib::systems
