#include "examples/goldilocks_models/controller/planned_traj_guard.h"

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using std::cout;
using std::endl;

namespace dairlib {
namespace goldilocks_models {

PlannedTrajGuard::PlannedTrajGuard() {
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();
  optimal_rom_traj_port_ =
      this->DeclareAbstractInputPort(
              "optimal_rom_traj",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  lipm_port_ =
      this->DeclareAbstractInputPort(
              "lipm_traj",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();

  ExponentialPlusPiecewisePolynomial<double> exp;
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  //  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("filtered_rom_traj", traj_inst,
                                  &PlannedTrajGuard::ApplyGuard);
}

void PlannedTrajGuard::ApplyGuard(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* output_traj) const {
  /// Read in optimal rom traj
  const auto& optimal_rom_traj =
      this->EvalAbstractInput(context, optimal_rom_traj_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();

  /// Seom computation
  bool no_available_traj = (context.get_time() > optimal_rom_traj.end_time());
  // TODO: connect this leafsystem to lcmsubscriber to get the status of solver
  bool planner_no_solution = (optimal_rom_traj.start_time() ==
                              -std::numeric_limits<double>::infinity());

  if (no_available_traj) {
    cout << "WARNING: no available traj! (traj start time, traj end time) = ("
         << optimal_rom_traj.start_time() << ", " << optimal_rom_traj.end_time()
         << "); current time = " << context.get_time() << "\n";
  }

  /// Apply the guard condition
  // If the solver didn't solve quick enough or the solution was not found, then
  // use the backup trajs
  //     if (no_available_traj || planner_no_solution) {
  if (false) {
    //  if (true) {
    //  std::cout << "Using backup controller\n";
    // Read in lipm traj
    const auto& lipm_traj =
        this->EvalAbstractInput(context, lipm_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();

    // Cast traj
    auto* output_traj_casted =
        (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
            ExponentialPlusPiecewisePolynomial<double>*>(output_traj);
    // Assign traj
    *output_traj_casted =
        (const ExponentialPlusPiecewisePolynomial<double>&)dynamic_cast<
            const ExponentialPlusPiecewisePolynomial<double>&>(lipm_traj);

    // Print message
    if (previous_traj_ != LIPM_TRAJ_INDEX) {
      if (no_available_traj) std::cout << "did't solve in time ";
      if (planner_no_solution) std::cout << "didn't find solution ";
      std::cout << ", switch to LIPM traj\n";
    }
    previous_traj_ = LIPM_TRAJ_INDEX;
  } else {
    //     std::cout << "Using optimal ROM controller\n";
    // Cast traj
    auto* output_traj_casted =
        (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
            ExponentialPlusPiecewisePolynomial<double>*>(output_traj);
    // Assign traj
    *output_traj_casted =
        (const ExponentialPlusPiecewisePolynomial<double>&)dynamic_cast<
            const ExponentialPlusPiecewisePolynomial<double>&>(
            optimal_rom_traj);

    // Print message
    if (previous_traj_ != OPTIMAL_ROM_TRAJ_INDEX) {
      std::cout << "Switch to optimal rom traj\n";
    }
    previous_traj_ = OPTIMAL_ROM_TRAJ_INDEX;
  }
}

}  // namespace goldilocks_models
}  // namespace dairlib
