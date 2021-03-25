#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib {

/// RomPlannerTrajectory is modified from DirconTrajectory

class RomPlannerTrajectory : public LcmTrajectory {
 public:
  // The lightweight flag is used for online lcm communication between processes
  RomPlannerTrajectory(const goldilocks_models::RomTrajOpt& trajopt,
                       const drake::solvers::MathematicalProgramResult& result,
                       const Eigen::VectorXd& quat_xyz_shift,
                       const std::string& name, const std::string& description,
                       bool lightweight = false, double time_shift = 0);

  explicit RomPlannerTrajectory(const lcmt_saved_traj& traj);
  explicit RomPlannerTrajectory(const std::string& filepath,
                                bool lightweight = false) {
    LoadFromFile(filepath, lightweight);
  }

  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory()
      const;
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory()
      const;

  /// Loads the saved state and input trajectory as well as the decision
  /// variables
  void LoadFromFile(const std::string& filepath, bool lightweight);

  int GetNumModes() const { return num_modes_; }

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
  /*Eigen::MatrixXd GetForceSamples(int mode) const {
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
  }*/
  Eigen::VectorXd GetDecisionVariables() const {
    return decision_vars_->datapoints;
  }

  const Trajectory* get_x0_FOM() const { return x0_FOM_; };
  const Trajectory* get_xf_FOM() const { return xf_FOM_; };
  const Eigen::VectorXd& get_stance_foot() const { return stance_foot_; };
  const Eigen::VectorXd& get_quat_xyz_shift() const { return quat_xyz_shift_; };

 private:
  static Eigen::VectorXd GetCollocationPoints(
      const Eigen::VectorXd& time_vector);
  int num_modes_ = 0;

  const Trajectory* decision_vars_;
  const Trajectory* u_;
  // std::vector<const Trajectory*> lambda_;
  // std::vector<const Trajectory*> lambda_c_;
  std::vector<const Trajectory*> x_;
  std::vector<const Trajectory*> xdot_;

  const Trajectory* x0_FOM_;
  const Trajectory* xf_FOM_;
  Eigen::VectorXd stance_foot_;
  Eigen::VectorXd quat_xyz_shift_;
  //  const Eigen::MatrixXd* stance_foot_;
  //  const Eigen::MatrixXd* quat_xyz_shift_;
};
}  // namespace dairlib
