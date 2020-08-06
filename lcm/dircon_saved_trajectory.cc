#include "dircon_saved_trajectory.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
using systems::trajectory_optimization::HybridDircon;

DirconTrajectory::DirconTrajectory(
    const MultibodyPlant<double>& plant,
    const systems::trajectory_optimization::HybridDircon<double>& dircon,
    const drake::solvers::MathematicalProgramResult& result,
    const std::string& name, const std::string& description) {
  num_modes_ = dircon.num_modes();

  LcmTrajectory::Trajectory decision_var_traj;
  decision_var_traj.traj_name = "decision_vars";
  decision_var_traj.datapoints = result.GetSolution();
  decision_var_traj.time_vector =
      VectorXd::Zero(decision_var_traj.datapoints.size());
  decision_var_traj.datatypes =
      vector<string>(decision_var_traj.datapoints.size());
  AddTrajectory(decision_var_traj.traj_name, decision_var_traj);
  decision_vars_ = &decision_var_traj;

  // State trajectory
  std::vector<Eigen::MatrixXd> x;
  std::vector<Eigen::MatrixXd> xdot;
  std::vector<Eigen::VectorXd> state_breaks;
  dircon.GetStateAndDerivativeSamples(result, &x, &xdot, &state_breaks);
  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory state_traj;
    LcmTrajectory::Trajectory state_derivative_traj;
    state_traj.traj_name = "state_traj" + std::to_string(mode);
    state_traj.datapoints = x[mode];
    state_traj.time_vector = state_breaks[mode];
    state_traj.datatypes = multibody::createStateNameVectorFromMap(plant);
    state_derivative_traj.traj_name =
        "state_derivative_traj" + std::to_string(mode);
    state_derivative_traj.datapoints = xdot[mode];
    state_derivative_traj.time_vector = state_breaks[mode];
    state_derivative_traj.datatypes =
        multibody::createStateNameVectorFromMap(plant);
    AddTrajectory(state_traj.traj_name, state_traj);
    AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);

    x_.push_back(&state_traj);
    xdot_.push_back(&state_derivative_traj);
  }

  // Input trajectory
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datapoints = dircon.GetInputSamples(result);
  input_traj.time_vector = dircon.GetSampleTimes(result);
  input_traj.datatypes = multibody::createActuatorNameVectorFromMap(plant);
  AddTrajectory(input_traj.traj_name, input_traj);
  u_ = &input_traj;

  has_data_ = true;
  ConstructMetadataObject(name, description);
}

PiecewisePolynomial<double> DirconTrajectory::ReconstructStateTrajectory() {
  DRAKE_DEMAND(has_data_);
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

PiecewisePolynomial<double> DirconTrajectory::ReconstructInputTrajectory() {
  DRAKE_DEMAND(has_data_);
  PiecewisePolynomial<double> input_traj =
      PiecewisePolynomial<double>::FirstOrderHold(u_->time_vector,
                                                  u_->datapoints);

  return input_traj;
}

void DirconTrajectory::LoadFromFile(const std::string& filepath) {
  LcmTrajectory::LoadFromFile(filepath);

  // Find all the state trajectories
  for (const auto& traj_name : GetTrajectoryNames()) {
    if (traj_name.find("state_traj") != std::string::npos) {
      ++num_modes_;
    }
  }
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(GetTrajectory("state_traj" + std::to_string(mode)));
    xdot_.push_back(
        GetTrajectory("state_derivative_traj" + std::to_string(mode)));
  }
  u_ = GetTrajectory("input_traj");
  decision_vars_ = GetTrajectory("decision_vars");

  has_data_ = true;
}

}  // namespace dairlib
