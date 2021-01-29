#pragma once
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;


namespace dairlib {
namespace centroidal_to {

enum class stance {
  kLeft = 0,
  kRight = 1,
  kDouble= 2
};

class CentroidalTrajOpt : public drake::solvers::MathematicalProgram {
 public:
  CentroidalTrajOpt(Eigen::Matrix3d inertia_tensor, double mass, double h,
                    double T_ss, double T_ds, double mu);
  void SetFinalPose(Eigen::Vector3d com, Eigen::Quaterniond pose);
  void SetFinalVel(Eigen::Vector3d v, Eigen::Vector3d omega);
  void SetFinalState(Eigen::VectorXd state);
  void SetInitialPose(Eigen::Vector3d com, Eigen::Quaterniond pose);
  void SetModeSequence(std::vector<stance> sequence, std::vector<double> times);
  void SetNominalStance(Eigen::Vector3d left, Eigen::Vector3d right);
  void SetMaxDeviationConstraint(Eigen::Vector3d max);
  void MakeImpulseFrictionConeConstraints();

 private:
  const Eigen::Matrix3d inertia_tensor_;
  std::vector<Eigen::Vector3d> nominal_stance_;
  std::vector<stance> sequence_;
  std::vector<double> times_;
  std::vector<drake::solvers::VectorXDecisionVariable> state_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> force_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> stance_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> impulse_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> post_impact_vars_;
  std::vector<bool> has_impact_;
  const double mass_;
  const double h_;
  const double T_ss_;
  const double T_ds_;
  const double mu_;
};
}
}


