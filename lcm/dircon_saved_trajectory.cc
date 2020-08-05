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
    const drake::multibody::MultibodyPlant<double>& plant)
    : plant_(plant),
      n_x_(plant.num_positions() + plant.num_velocities()),
      n_u_(plant.num_actuators()) {
  has_data_ = false;
}

DirconTrajectory::DirconTrajectory(
    const MultibodyPlant<double>& plant,
    const systems::trajectory_optimization::HybridDircon<double>& dircon,
    const drake::solvers::MathematicalProgramResult& result,
    const std::string& name, const std::string& description)
    : DirconTrajectory(plant) {
  num_modes_ = dircon.num_modes();
  mode_lengths_ = dircon.mode_lengths();

  decision_vars_ = result.GetSolution();
  h_ = dircon.GetSampleTimes(result);
  u_ = dircon.GetInputSamples(result);

  dircon.GetStateAndDerivativeSamples(result, x_, xdot_, state_breaks_);

  has_data_ = true;
  ConstructMetadataObject(name, description);
}

PiecewisePolynomial<double> DirconTrajectory::ReconstructStateTrajectory() {
  DRAKE_ASSERT(has_data_);
  PiecewisePolynomial<double> state_traj =
      PiecewisePolynomial<double>::CubicHermite(state_breaks_[0], x_[0],
                                                xdot_[0]);

  for (int mode = 1; mode < num_modes_; ++mode) {
    // Cannot form trajectory with only a single break
    if(mode_lengths_[mode] < 2){
      continue;
    }
    state_traj.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        state_breaks_[mode], x_[mode], xdot_[mode]));
  }
  return state_traj;
}

PiecewisePolynomial<double> DirconTrajectory::ReconstructInputTrajectory() {
  DRAKE_ASSERT(has_data_);
  PiecewisePolynomial<double> input_traj =
      PiecewisePolynomial<double>::FirstOrderHold(h_, u_);

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
  mode_lengths_.resize(num_modes_);
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(GetTrajectory("state_traj" + std::to_string(mode)).datapoints);
    xdot_.push_back(
        GetTrajectory("state_derivative_traj" + std::to_string(mode))
            .datapoints);
    state_breaks_.push_back(
        GetTrajectory("state_traj" + std::to_string(mode)).time_vector);
    mode_lengths_[mode] = state_breaks_[mode].size();
  }
  u_ = GetTrajectory("input_traj").datapoints;
  h_ = GetTrajectory("input_traj").time_vector;
  decision_vars_ = GetTrajectory("decision_vars").datapoints;

  has_data_ = true;
}

void DirconTrajectory::WriteToFile(const std::string& filepath) {
  // Decision variables
  LcmTrajectory::Trajectory decision_vars;
  decision_vars.traj_name = "decision_vars";
  decision_vars.datapoints = decision_vars_;
  decision_vars.time_vector = VectorXd::Zero(decision_vars.datapoints.size());
  decision_vars.datatypes = vector<string>(decision_vars.datapoints.size());
  AddTrajectory(decision_vars.traj_name, decision_vars);

  // State trajectory
  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory state_traj;
    LcmTrajectory::Trajectory state_derivative_traj;
    state_traj.traj_name = "state_traj" + std::to_string(mode);
    state_traj.datapoints = x_[mode];
    state_traj.time_vector = state_breaks_[mode];
    state_traj.datatypes = multibody::createStateNameVectorFromMap(plant_);
    state_derivative_traj.traj_name =
        "state_derivative_traj" + std::to_string(mode);
    state_derivative_traj.datapoints = xdot_[mode];
    state_derivative_traj.time_vector = state_breaks_[mode];
    state_derivative_traj.datatypes =
        multibody::createStateNameVectorFromMap(plant_);
    AddTrajectory(state_traj.traj_name, state_traj);
    AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);
  }

  // Input trajectory
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datapoints = u_;
  input_traj.time_vector = h_;
  input_traj.datatypes = multibody::createActuatorNameVectorFromMap(plant_);
  AddTrajectory(input_traj.traj_name, input_traj);

  // Final write
  LcmTrajectory::WriteToFile(filepath);
}

}  // namespace dairlib
