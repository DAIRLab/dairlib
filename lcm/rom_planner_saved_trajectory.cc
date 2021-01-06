#include "rom_planner_saved_trajectory.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
using goldilocks_models::RomTrajOpt;

RomPlannerTrajectory::RomPlannerTrajectory(
    const RomTrajOpt& trajopt,
    const drake::solvers::MathematicalProgramResult& result,
    const std::string& name, const std::string& description) {
  num_modes_ = trajopt.num_modes();

  // Create state and input names
  const auto& rom = trajopt.reduced_order_model();
  std::vector<string> state_names;
  std::vector<string> input_names;
  for (int i = 0; i < rom.n_y() + rom.n_yddot(); i++) {
    state_names.push_back("state" + std::to_string(i));
  }
  for (int i = 0; i < rom.n_tau(); i++) {
    input_names.push_back("input" + std::to_string(i));
  }

  // State trajectory
  std::vector<Eigen::MatrixXd> x;
  std::vector<Eigen::MatrixXd> xdot;
  std::vector<Eigen::VectorXd> state_breaks;
  trajopt.GetStateAndDerivativeSamples(result, &x, &xdot, &state_breaks);
  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory state_traj;
    LcmTrajectory::Trajectory state_derivative_traj;
    // LcmTrajectory::Trajectory force_traj;

    state_traj.traj_name = "state_traj" + std::to_string(mode);
    state_traj.datapoints = x[mode];
    state_traj.time_vector = state_breaks[mode];
    state_traj.datatypes = state_names;

    state_derivative_traj.traj_name =
        "state_derivative_traj" + std::to_string(mode);
    state_derivative_traj.datapoints = xdot[mode];
    state_derivative_traj.time_vector = state_breaks[mode];
    state_derivative_traj.datatypes = state_names;

    /*// Force vars
    force_traj.traj_name = "force_vars" + std::to_string(mode);
    std::vector<std::string> force_names;
    std::vector<std::string> collocation_force_names;
    int num_forces = 0;
    for (int i = 0; i < trajopt.num_kinematic_constraints_wo_skipping(mode);
         ++i) {
      force_names.push_back("lambda_" + std::to_string(num_forces));
      collocation_force_names.push_back("lambda_c_" +
                                        std::to_string(num_forces));
      ++num_forces;
    }
    force_traj.traj_name = "force_vars" + std::to_string(mode);
    force_traj.time_vector = state_breaks[mode];
    force_traj.datapoints =
        Map<MatrixXd>(result.GetSolution(trajopt.force_vars(mode)).data(),
                      num_forces, force_traj.time_vector.size());
    force_traj.datatypes = force_names;

    // Collocation force vars
    if (state_breaks[mode].size() > 1) {
      LcmTrajectory::Trajectory collocation_force_traj;
      collocation_force_traj.traj_name =
          "collocation_force_vars" + std::to_string(mode);
      collocation_force_traj.datatypes = collocation_force_names;
      collocation_force_traj.time_vector =
          GetCollocationPoints(state_breaks[mode]);
      collocation_force_traj.datapoints = Map<MatrixXd>(
          result.GetSolution(trajopt.collocation_force_vars(mode)).data(),
          num_forces, collocation_force_traj.time_vector.size());
      AddTrajectory(collocation_force_traj.traj_name, collocation_force_traj);
      lambda_c_.push_back(&collocation_force_traj);
    }*/

    AddTrajectory(state_traj.traj_name, state_traj);
    AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);
    // AddTrajectory(force_traj.traj_name, force_traj);

    x_.push_back(&state_traj);
    xdot_.push_back(&state_derivative_traj);
    // lambda_.push_back(&force_traj);
  }

  // Input trajectory
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datapoints = trajopt.GetInputSamples(result);
  input_traj.time_vector = trajopt.GetSampleTimes(result);
  input_traj.datatypes = input_names;
  AddTrajectory(input_traj.traj_name, input_traj);
  u_ = &input_traj;

  // Decision variables
  LcmTrajectory::Trajectory decision_var_traj;
  decision_var_traj.traj_name = "decision_vars";
  decision_var_traj.datapoints = result.GetSolution();
  decision_var_traj.time_vector = VectorXd::Zero(1);
  decision_var_traj.datatypes =
      vector<string>(decision_var_traj.datapoints.size());
  for (int i = 0; i < decision_var_traj.datapoints.size(); i++) {
    decision_var_traj.datatypes[i] = trajopt.decision_variable(i).get_name();
  }
  AddTrajectory(decision_var_traj.traj_name, decision_var_traj);
  decision_vars_ = &decision_var_traj;

  ConstructMetadataObject(name, description);
}

PiecewisePolynomial<double> RomPlannerTrajectory::ReconstructStateTrajectory()
    const {
  PiecewisePolynomial<double> state_traj =
      PiecewisePolynomial<double>::CubicHermite(
          x_[0]->time_vector, x_[0]->datapoints, xdot_[0]->datapoints);

  for (int mode = 1; mode < num_modes_; ++mode) {
    // Cannot form trajectory with only a single break
    if (x_[mode]->time_vector.size() < 2) {
      continue;
    }
    state_traj.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        x_[mode]->time_vector, x_[mode]->datapoints, xdot_[mode]->datapoints));
  }
  return state_traj;
}

PiecewisePolynomial<double> RomPlannerTrajectory::ReconstructInputTrajectory()
    const {
  PiecewisePolynomial<double> input_traj =
      PiecewisePolynomial<double>::FirstOrderHold(u_->time_vector,
                                                  u_->datapoints);

  return input_traj;
}

void RomPlannerTrajectory::LoadFromFile(const std::string& filepath) {
  LcmTrajectory::LoadFromFile(filepath);

  // Find all the state trajectories
  for (const auto& traj_name : GetTrajectoryNames()) {
    if (traj_name.find("state_traj") != std::string::npos) {
      ++num_modes_;
    }
  }
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(&GetTrajectory("state_traj" + std::to_string(mode)));
    xdot_.push_back(
        &GetTrajectory("state_derivative_traj" + std::to_string(mode)));
    /*lambda_.push_back(&GetTrajectory("force_vars" + std::to_string(mode)));
    if (x_[mode]->time_vector.size() > 1) {
      lambda_c_.push_back(
          &GetTrajectory("collocation_force_vars" + std::to_string(mode)));
    }*/
  }
  u_ = &GetTrajectory("input_traj");
  decision_vars_ = &GetTrajectory("decision_vars");
}

Eigen::VectorXd RomPlannerTrajectory::GetCollocationPoints(
    const Eigen::VectorXd& time_vector) {
  // using a + (b - a) / 2 midpoint
  int num_knotpoints = time_vector.size();
  return time_vector.head(num_knotpoints - 1) +
         0.5 * (time_vector.tail(num_knotpoints - 1) -
                time_vector.head(num_knotpoints - 1));
}

}  // namespace dairlib
