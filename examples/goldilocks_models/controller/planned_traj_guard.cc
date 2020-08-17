#include "examples/goldilocks_models/controller/planned_traj_guard.h"

using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

PlannedTrajGuard::PlannedTrajGuard(double max_solve_time)
    : max_solve_time_(max_solve_time) {
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
  optimal_traj_port_ =
      this->DeclareAbstractInputPort(
              traj_name,
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  lipm_port_ =
      this->DeclareAbstractInputPort(
              traj_name,
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("filtered_rom_traj", traj_inst,
                                  &PlannedTrajGuard::ApplyGuard);
};

void PlannedTrajGuard::ApplyGuard(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* output_traj) const {
  // Cast traj
  auto* output_traj_casted =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output_traj);

  // Read in optimal rom traj
  const auto& optimal_rom_traj =
      this->EvalAbstractInput(context, optimal_traj_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();

  // Apply the guard condition
  // If the solve time is longer than max_solve_time or the solution was not
  // found, then use lipm traj
  if ((context.get_time() - optimal_rom_traj.time_vec.at(0) >
       max_solve_time_) ||
      (optimal_rom_traj.trajectory_name == "failed")) {
    // Read in lipm traj
    const auto& lipm_traj =
        this->EvalAbstractInput(context, lipm_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();

    // Assign traj
    *output_traj_casted = lipm_traj;
  } else {
    // Assign traj
    *output_traj_casted = optimal_rom_traj;
  }
};

}  // namespace goldilocks_models
}  // namespace dairlib
