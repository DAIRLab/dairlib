#include "dircon_saved_trajectory.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::HybridDircon;

DirconTrajectory::DirconTrajectory(
    const MultibodyPlant<double>& plant,
    const systems::trajectory_optimization::Dircon<double>& dircon,
    const drake::solvers::MathematicalProgramResult& result,
    const std::string& name, const std::string& description) {
  num_modes_ = dircon.num_modes();

  // State trajectory
  std::vector<Eigen::MatrixXd> x;
  std::vector<Eigen::MatrixXd> xdot;
  std::vector<Eigen::VectorXd> state_breaks;
  dircon.GetStateAndDerivativeSamples(result, &x, &xdot, &state_breaks);

  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory state_traj;
    LcmTrajectory::Trajectory state_derivative_traj;
    LcmTrajectory::Trajectory force_traj;

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

    // Force vars
    force_traj.traj_name = "force_vars" + std::to_string(mode);
    std::vector<std::string> force_names;
    std::vector<std::string> collocation_force_names;

    int num_forces = dircon.get_evaluator_set(mode).count_full();
    for (int i = 0; i < num_forces; ++i) {
      force_names.push_back("lambda_" + std::to_string(i));
      collocation_force_names.push_back("lambda_c_" + std::to_string(i));
    }
    force_traj.datatypes = force_names;
    force_traj.time_vector = state_breaks[mode];
    force_traj.datapoints =
        Eigen::Map<MatrixXd>(dircon.GetForceSamplesByMode(result, mode).data(),
                             num_forces, force_traj.time_vector.size());

    // Collocation force vars
    if (state_breaks[mode].size() > 1) {
      LcmTrajectory::Trajectory collocation_force_traj;
      collocation_force_traj.traj_name =
          "collocation_force_vars" + std::to_string(mode);
      collocation_force_traj.datatypes = collocation_force_names;
      collocation_force_traj.time_vector =
          GetCollocationPoints(state_breaks[mode]);
      collocation_force_traj.datapoints =
          MatrixXd::Zero(num_forces, collocation_force_traj.time_vector.size());
      for (int i = 0; i < collocation_force_traj.time_vector.size(); ++i) {
        collocation_force_traj.datapoints.col(i) =
            result.GetSolution(dircon.collocation_force_vars(mode, i));
      }
      AddTrajectory(collocation_force_traj.traj_name, collocation_force_traj);
      lambda_c_.push_back(&collocation_force_traj);
    }

    AddTrajectory(state_traj.traj_name, state_traj);
    AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);
    AddTrajectory(force_traj.traj_name, force_traj);

    x_.push_back(&state_traj);
    xdot_.push_back(&state_derivative_traj);
    lambda_.push_back(&force_traj);
  }

  // Input trajectory
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datapoints = dircon.GetInputSamples(result);
  input_traj.time_vector = dircon.GetSampleTimes(result);
  input_traj.datatypes = multibody::createActuatorNameVectorFromMap(plant);
  AddTrajectory(input_traj.traj_name, input_traj);
  u_ = &input_traj;

  // Decision variables
  LcmTrajectory::Trajectory decision_var_traj;
  decision_var_traj.traj_name = "decision_vars";
  decision_var_traj.datapoints = result.GetSolution();
  decision_var_traj.time_vector =
      VectorXd::Zero(1);
  decision_var_traj.datatypes =
      vector<string>(decision_var_traj.datapoints.size());
  AddTrajectory(decision_var_traj.traj_name, decision_var_traj);
  decision_vars_ = &decision_var_traj;

  ConstructMetadataObject(name, description);
}

// TODO(yangwill): Duplicate code, delete when we transition to a single Dircon
DirconTrajectory::DirconTrajectory(
    const MultibodyPlant<double>& plant,
    const systems::trajectory_optimization::HybridDircon<double>& dircon,
    const drake::solvers::MathematicalProgramResult& result,
    const std::string& name, const std::string& description) {
  num_modes_ = dircon.num_modes();

  // State trajectory
  std::vector<Eigen::MatrixXd> x;
  std::vector<Eigen::MatrixXd> xdot;
  std::vector<Eigen::VectorXd> state_breaks;
  dircon.GetStateAndDerivativeSamples(result, &x, &xdot, &state_breaks);
  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory state_traj;
    LcmTrajectory::Trajectory state_derivative_traj;
    LcmTrajectory::Trajectory force_traj;

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

    // Force vars
    force_traj.traj_name = "force_vars" + std::to_string(mode);
    std::vector<std::string> force_names;
    std::vector<std::string> collocation_force_names;
    int num_forces = 0;
    for (int i = 0; i < dircon.num_kinematic_constraints_wo_skipping(mode);
         ++i) {
      force_names.push_back("lambda_" + std::to_string(num_forces));
      collocation_force_names.push_back("lambda_c_" +
                                        std::to_string(num_forces));
      ++num_forces;
    }
    force_traj.traj_name = "force_vars" + std::to_string(mode);
    force_traj.time_vector = state_breaks[mode];
    force_traj.datapoints =
        Map<MatrixXd>(result.GetSolution(dircon.force_vars(mode)).data(),
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
          result.GetSolution(dircon.collocation_force_vars(mode)).data(),
          num_forces, collocation_force_traj.time_vector.size());
      AddTrajectory(collocation_force_traj.traj_name, collocation_force_traj);
      lambda_c_.push_back(&collocation_force_traj);
    }

    AddTrajectory(state_traj.traj_name, state_traj);
    AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);
    AddTrajectory(force_traj.traj_name, force_traj);

    x_.push_back(&state_traj);
    xdot_.push_back(&state_derivative_traj);
    lambda_.push_back(&force_traj);
  }

  // Input trajectory
  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datapoints = dircon.GetInputSamples(result);
  input_traj.time_vector = dircon.GetSampleTimes(result);
  input_traj.datatypes = multibody::createActuatorNameVectorFromMap(plant);
  AddTrajectory(input_traj.traj_name, input_traj);
  u_ = &input_traj;

  // Decision variables
  LcmTrajectory::Trajectory decision_var_traj;
  decision_var_traj.traj_name = "decision_vars";
  decision_var_traj.datapoints = result.GetSolution();
  decision_var_traj.time_vector =
      VectorXd::Zero(1);
  decision_var_traj.datatypes =
      vector<string>(decision_var_traj.datapoints.size());
  AddTrajectory(decision_var_traj.traj_name, decision_var_traj);
  decision_vars_ = &decision_var_traj;

  ConstructMetadataObject(name, description);
}

PiecewisePolynomial<double> DirconTrajectory::ReconstructStateTrajectory()
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

PiecewisePolynomial<double> DirconTrajectory::ReconstructInputTrajectory()
    const {
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
    x_.push_back(&GetTrajectory("state_traj" + std::to_string(mode)));
    xdot_.push_back(
        &GetTrajectory("state_derivative_traj" + std::to_string(mode)));
    lambda_.push_back(&GetTrajectory("force_vars" + std::to_string(mode)));
    if (x_[mode]->time_vector.size() > 1) {
      lambda_c_.push_back(
          &GetTrajectory("collocation_force_vars" + std::to_string(mode)));
    }
  }
  u_ = &GetTrajectory("input_traj");
  decision_vars_ = &GetTrajectory("decision_vars");
}

Eigen::VectorXd DirconTrajectory::GetCollocationPoints(
    const Eigen::VectorXd& time_vector) {
  // using a + (b - a) / 2 midpoint
  int num_knotpoints = time_vector.size();
  return time_vector.head(num_knotpoints - 1) +
      0.5 * (time_vector.tail(num_knotpoints - 1) -
          time_vector.head(num_knotpoints - 1));
}

}  // namespace dairlib
