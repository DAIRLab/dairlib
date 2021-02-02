#include "examples/goldilocks_models/controller/rom_traj_receiver.h"

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

OptimalRoMTrajReceiver::OptimalRoMTrajReceiver() {
  rom_traj_lcm_port_ =
      this->DeclareAbstractInputPort("rom_traj_lcm",
                                     drake::Value<dairlib::lcmt_saved_traj>{})
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  ExponentialPlusPiecewisePolynomial<double> exp;
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  //  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("rom_traj", traj_inst,
                                  &OptimalRoMTrajReceiver::CalcDesiredTraj);
};

void OptimalRoMTrajReceiver::CalcDesiredTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj
  auto* traj_casted = (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
      ExponentialPlusPiecewisePolynomial<double>*>(traj);

  // Construct traj from lcm message
  LcmTrajectory traj_data(*(this->EvalInputValue<dairlib::lcmt_saved_traj>(
      context, rom_traj_lcm_port_)));
  const auto& traj_names = traj_data.GetTrajectoryNames();

  int n_traj = traj_names.size();
  int n_y = traj_data.GetTrajectory(traj_names[0]).datatypes.size() / 2;

  // TODO(yminchen): currently we construct ExponentialPlusPiecewisePolynomial
  //  although the traj is PiecewisePolynomial, because we need to use this with
  //  lipm in the guard.
  //  One solution to this could be not sending Trajectory class but BasicVector
  //  to OSC (change the API)
  //  Put up an issue for this.
  const LcmTrajectory::Trajectory& traj0 =
      traj_data.GetTrajectory(traj_names[0]);
  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicHermite(
          traj0.time_vector, traj0.datapoints.topRows(n_y),
          traj0.datapoints.bottomRows(n_y));
  for (int mode = 1; mode < n_traj; ++mode) {
    const LcmTrajectory::Trajectory& traj_i =
        traj_data.GetTrajectory(traj_names[mode]);
    pp_part.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        traj_i.time_vector, traj_i.datapoints.topRows(n_y),
        traj_i.datapoints.bottomRows(n_y)));
  }

  // Assign traj
  *traj_casted = ExponentialPlusPiecewisePolynomial<double>(pp_part);
};

}  // namespace goldilocks_models
}  // namespace dairlib
