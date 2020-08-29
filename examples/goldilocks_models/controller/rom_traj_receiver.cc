#include "examples/goldilocks_models/controller/rom_traj_receiver.h"

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

OptimalRoMTrajReceiver::OptimalRoMTrajReceiver() {
  rom_traj_lcm_port_ =
      this->DeclareAbstractInputPort(
              "rom_traj_lcm", drake::Value<dairlib::lcmt_trajectory_block>{})
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp_part(VectorXd(0));
  Eigen::MatrixXd K = Eigen::MatrixXd::Ones(0, 0);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(0, 0);
  Eigen::MatrixXd alpha = Eigen::MatrixXd::Ones(0, 0);
  ExponentialPlusPiecewisePolynomial<double> exp(K, A, alpha, pp_part);
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

  // TODO(yminchen): test how much time it takes to construct traj below. if
  //  it's too long, you can avoid copying it in every loop

  // Construct traj from lcm message
  LcmTrajectory::Trajectory traj_data(
      "" /*tra_name*/, *(this->EvalInputValue<dairlib::lcmt_trajectory_block>(
                           context, rom_traj_lcm_port_)));
  int n_y = traj_data.datatypes.size() / 2;

  // TODO(yminchen): currently we construct ExponentialPlusPiecewisePolynomial
  //  although the traj is PiecewisePolynomial, because we need to use this with
  //  lipm in the guard.
  //  One solution to this could be not sending Trajectory class but BasicVector
  //  to OSC (change the API)
  //  Put up an issue for this.
  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicHermite(
          traj_data.time_vector, traj_data.datapoints.topRows(n_y),
          traj_data.datapoints.bottomRows(n_y));

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(traj_data.datatypes.size(), 0);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(0, 0);
  Eigen::MatrixXd alpha =
      Eigen::MatrixXd::Zero(0, traj_data.time_vector.size() - 1);

  // Assign traj
  *traj_casted =
      ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, pp_part);
};

}  // namespace goldilocks_models
}  // namespace dairlib
