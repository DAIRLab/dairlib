#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/goldilocks_models/planning/rom_traj_opt.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib {

/// RomPlannerTrajectory is modified from DirconTrajectory

class RomPlannerTrajectory : public LcmTrajectory {
 public:
  RomPlannerTrajectory& operator=(const RomPlannerTrajectory& old);

  // The lightweight flag is used for online lcm communication between processes
  RomPlannerTrajectory(const goldilocks_models::RomTrajOpt& trajopt,
                       const drake::solvers::MathematicalProgramResult& result,
                       const Eigen::MatrixXd& x0_global,
                       const Eigen::MatrixXd& xf_global,
                       const std::string& name, const std::string& description,
                       bool lightweight = false, double current_time = 0);

  explicit RomPlannerTrajectory(const lcmt_timestamped_saved_traj& traj);
  explicit RomPlannerTrajectory(const std::string& filepath,
                                bool lightweight = false) {
    LoadFromFile(filepath, lightweight);
  }

  RomPlannerTrajectory(){};

  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory()
      const;
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory()
      const;

  lcmt_timestamped_saved_traj GenerateLcmObject() const;

  /// Loads the saved state and input trajectory as well as the decision
  /// variables
  void LoadFromFile(const std::string& filepath, bool lightweight);
  using LcmTrajectory::LoadFromFile;  // get rid of the compiler warning

  int get_utime() const { return utime_;};
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
  Eigen::VectorXd GetStateBreaks(int mode) const {
    DRAKE_DEMAND(mode >= 0);
    DRAKE_DEMAND(mode < num_modes_);
    return x_[mode]->time_vector;
  }
  Eigen::MatrixXd GetInputSamples() const { return u_->datapoints; }
  Eigen::MatrixXd GetBreaks() const { return u_->time_vector; }
  Eigen::VectorXd GetDecisionVariables() const {
    return decision_vars_->datapoints;
  }

  const Eigen::MatrixXd& get_x0() const { return global_x0_FOM_->datapoints; };
  const Eigen::VectorXd& get_x0_time() const {
    return global_x0_FOM_->time_vector;
  };
  const Eigen::MatrixXd& get_xf() const { return global_xf_FOM_->datapoints; };
  const Eigen::VectorXd& get_xf_time() const {
    return global_xf_FOM_->time_vector;
  };
  const Eigen::VectorXd& get_stance_foot() const { return stance_foot_; };

  drake::trajectories::PiecewisePolynomial<double> ConstructPositionTrajectory()
      const;

 private:
  static Eigen::VectorXd GetCollocationPoints(
      const Eigen::VectorXd& time_vector);
  int utime_;
  int num_modes_ = 0;

  const Trajectory* decision_vars_;
  const Trajectory* u_;
  std::vector<const Trajectory*> x_;
  std::vector<const Trajectory*> xdot_;

  const Trajectory* global_x0_FOM_;
  const Trajectory* global_xf_FOM_;
  Eigen::VectorXd stance_foot_;
  //  const Eigen::MatrixXd* stance_foot_;
};
}  // namespace dairlib
