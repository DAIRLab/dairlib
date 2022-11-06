#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/goldilocks_models/planning/hybrid_rom_traj_opt.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib {

/// HybridRomPlannerTrajectory is modified from RomPlannerTrajectory

class HybridRomPlannerTrajectory : public LcmTrajectory {
 public:
  HybridRomPlannerTrajectory& operator=(const HybridRomPlannerTrajectory& old);

  // The lightweight flag is used for online lcm communication between processes
  HybridRomPlannerTrajectory(
      const goldilocks_models::HybridRomTrajOpt& trajopt,
      const drake::solvers::MathematicalProgramResult& result,
      const Eigen::MatrixXd& global_feet_pos,
      const Eigen::MatrixXd& global_com_pos,
      const Eigen::VectorXd& current_quat_xyz_shift,
      const Eigen::VectorXd& current_stance_foot_pos, const std::string& name,
      const std::string& description, bool lightweight = false,
      double current_time = 0);

  explicit HybridRomPlannerTrajectory(const lcmt_timestamped_saved_traj& traj);
  explicit HybridRomPlannerTrajectory(const std::string& filepath,
                                      bool lightweight = false) {
    LoadFromFile(filepath, lightweight);
  }

  HybridRomPlannerTrajectory(){};

  drake::trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory()
      const;
  drake::trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory()
      const;

  lcmt_timestamped_saved_traj GenerateLcmObject() const;

  /// Loads the saved state and input trajectory as well as the decision
  /// variables
  void LoadFromFile(const std::string& filepath, bool lightweight);
  using LcmTrajectory::LoadFromFile;  // get rid of the compiler warning

  int get_utime() const { return utime_; };
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
  Eigen::VectorXd GetBreaks() const { return u_->time_vector; }
  Eigen::VectorXd GetDecisionVariables() const {
    return decision_vars_->datapoints;
  }

  const Eigen::MatrixXd& get_global_feet_pos() const {
    return global_feet_pos_->datapoints;
  };
  const Eigen::VectorXd& get_global_feet_pos_time() const {
    return global_feet_pos_->time_vector;
  };
  const Eigen::MatrixXd& get_global_com_pos() const {
    return global_com_pos_->datapoints;
  };
  const Eigen::VectorXd& get_global_com_pos_time() const {
    return global_com_pos_->time_vector;
  };
  /*const Eigen::VectorXd& get_stance_foot() const { return stance_foot_; };
  const Eigen::VectorXd& get_current_quat_xyz_shift() const {
    return current_quat_xyz_shift_;
  };
  const Eigen::VectorXd& get_current_stance_foot_pos() const {
    return current_stance_foot_pos_;
  };*/
  Eigen::VectorXd get_stance_foot() const {
    return GetTrajectory("stance_foot").datapoints.row(0).transpose();
  };
  Eigen::VectorXd get_current_quat_xyz_shift() const {
    return GetTrajectory("current_quat_xyz_shift")
        .datapoints.row(0)
        .transpose();
  };
  Eigen::VectorXd get_current_stance_foot_pos() const {
    return GetTrajectory("current_stance_foot_pos")
        .datapoints.row(0)
        .transpose();
  };

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

  const Trajectory* global_feet_pos_;
  const Trajectory* global_com_pos_;
  //  Eigen::VectorXd stance_foot_;
  //  //  const Eigen::MatrixXd* stance_foot_;
  //  Eigen::VectorXd current_quat_xyz_shift_;
  //  Eigen::VectorXd current_stance_foot_pos_;
};

/// ReadHybridRomPlannerTrajectory
drake::trajectories::PiecewisePolynomial<double> ReadHybridRomPlannerTrajectory(
    const std::string& path, bool offset_time_to_zero = false);

}  // namespace dairlib
