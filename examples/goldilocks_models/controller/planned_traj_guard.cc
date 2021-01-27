#include "examples/goldilocks_models/controller/planned_traj_guard.h"

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

PlannedTrajGuard::PlannedTrajGuard(double max_solve_time)
    : max_solve_time_(max_solve_time) {
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

  PiecewisePolynomial<double> pp_part(VectorXd(0));
  Eigen::MatrixXd K = Eigen::MatrixXd::Ones(0, 0);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(0, 0);
  Eigen::MatrixXd alpha = Eigen::MatrixXd::Ones(0, 0);
  ExponentialPlusPiecewisePolynomial<double> exp(K, A, alpha, pp_part);
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  //  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("filtered_rom_traj", traj_inst,
                                  &PlannedTrajGuard::ApplyGuard);
}

void PlannedTrajGuard::ApplyGuard(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* output_traj) const {
  // Read in optimal rom traj
  const auto& optimal_rom_traj =
      this->EvalAbstractInput(context, optimal_rom_traj_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();

  // Update previous message's arrival time (the time when the planner finished
  // planning)
  if (optimal_rom_traj.start_time() != prev_traj_start_time_) {
    prev_message_arrival_time_ = context.get_time();
    prev_traj_start_time_ = optimal_rom_traj.start_time();
  }

  // TODO: also need to consider that when the current time is larger than the
  //  end of the traj time (we fall back to lipm traj in this case)
  bool planner_timeout =
      (context.get_time() - prev_message_arrival_time_ > max_solve_time_);
  bool planner_no_solution = (optimal_rom_traj.start_time() ==
                              -std::numeric_limits<double>::infinity());

  // Apply the guard condition
  // If the solve time is longer than max_solve_time or the solution was not
  // found, then use the backup trajs
  // TODO: currently always use LIPM traj to debug the controller. (there is a
  //  bug in the original LIPM control)
  //  The bug could be that we are using RomTrackingData to tracking LIPM traj,
  //  but the ROM is actually using COM wrt feet. Therefore, we just need to not
  //  use this RomTrackingData LIPM
//     if (planner_timeout || planner_no_solution) {
//    if (false) {
  if (true) {
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
      if (planner_timeout) std::cout << "did't solve in time ";
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
