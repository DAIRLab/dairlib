#include "examples/goldilocks_models/controller/rom_traj_receiver.h"

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
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("rom_traj", traj_inst,
                                  &OptimalRoMTrajReceiver::CalcDesiredTraj);
};

void OptimalRoMTrajReceiver::CalcDesiredTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj
  auto* traj_casted =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  // TODO(yminchen): test how much time it takes to construct traj below. if
  //  it's too long, you can avoid copying it in every loop

  // Construct traj from lcm message
  LcmTrajectory::Trajectory traj_data(
      "" /*tra_name*/, *(this->EvalInputValue<dairlib::lcmt_trajectory_block>(
                           context, rom_traj_lcm_port_)));
  int n_y = traj_data.datatypes.size() / 2;
  *traj_casted = PiecewisePolynomial<double>::CubicHermite(
      traj_data.time_vector, traj_data.datapoints.topRows(n_y),
      traj_data.datapoints.bottomRows(n_y));
};

}  // namespace goldilocks_models
}  // namespace dairlib
