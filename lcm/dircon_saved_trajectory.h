#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "lcm/lcm_trajectory.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

namespace dairlib {

/// Used for saving/loading DIRCON trajectories.
/// To save a DirconTrajectory, create a DirconTrajectory object (second
/// constructor) with the solution from dircon, then call WriteToFile() with the
/// desired filepath
///
/// To load a saved DirconTrajectory object, create an empty DirconTrajectory
/// object (first constructor) and call the LoadFromFile() with relative
/// filepath of the previously saved DirconTrajectory object

/// DirconTrajectory by default contains four Trajectory objects: the state
/// trajectory, the input trajectory, the force trajectory, and the decision
/// variables. Additional trajectories can be added using the AddTrajectory()
/// function

class DirconTrajectory : public LcmTrajectory {
 public:
  DirconTrajectory(const std::string& filepath) { LoadFromFile(filepath); }

  DirconTrajectory(
      const drake::multibody::MultibodyPlant<double>& plant,
      const systems::trajectory_optimization::Dircon<double>& dircon,
      const drake::solvers::MathematicalProgramResult& result,
      const std::string& name, const std::string& description);

  DirconTrajectory(
      const drake::multibody::MultibodyPlant<double>& plant,
      const systems::trajectory_optimization::HybridDircon<double>& dircon,
      const drake::solvers::MathematicalProgramResult& result,
      const std::string& name, const std::string& description);

  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory()
      const;
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory()
      const;

  /// Loads the saved state and input trajectory as well as the decision
  /// variables
  void LoadFromFile(const std::string& filepath) override;

  Eigen::MatrixXd GetStateSamples(int mode) const {
    DRAKE_DEMAND(mode >= 0);
    DRAKE_DEMAND(mode < num_modes_);
    return x_[mode]->datapoints;
  }
  Eigen::MatrixXd GetStateDerivativeSamples(int mode) const {
    DRAKE_DEMAND(mode >= 0);
    DRAKE_DEMAND(mode < num_modes_);
    return xdot_[mode]->datapoints;
  }
  Eigen::MatrixXd GetStateBreaks(int mode) const {
    DRAKE_DEMAND(mode >= 0);
    DRAKE_DEMAND(mode < num_modes_);
    return x_[mode]->time_vector;
  }
  Eigen::MatrixXd GetInputSamples() const { return u_->datapoints; }
  Eigen::MatrixXd GetBreaks() const { return u_->time_vector; }
  Eigen::MatrixXd GetForceSamples(int mode) const {
    return lambda_[mode]->datapoints;
  }
  Eigen::MatrixXd GetForceBreaks(int mode) const {
    return lambda_[mode]->time_vector;
  }
  Eigen::MatrixXd GetCollocationForceSamples(int mode) const {
    return lambda_c_[mode]->datapoints;
  }
  Eigen::MatrixXd GetCollocationForceBreaks(int mode) const {
    return lambda_c_[mode]->time_vector;
  }
  Eigen::VectorXd GetDecisionVariables() const {
    return decision_vars_->datapoints;
  }

  int GetNumModes() const { return num_modes_; }

 private:
  static Eigen::VectorXd GetCollocationPoints(
      const Eigen::VectorXd& time_vector);
  int num_modes_ = 0;

  const Trajectory* decision_vars_;
  const Trajectory* u_;
  std::vector<const Trajectory*> lambda_;
  std::vector<const Trajectory*> lambda_c_;
  std::vector<const Trajectory*> x_;
  std::vector<const Trajectory*> xdot_;
};
}  // namespace dairlib
