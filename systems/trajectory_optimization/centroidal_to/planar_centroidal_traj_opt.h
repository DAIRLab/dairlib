#pragma once
#include <drake/solvers/mathematical_program_result.h>
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
namespace planar {

enum stance {
  L = 0,
  R = 1,
  D = 2
};

typedef struct CentroidalMode {
  std::vector<drake::solvers::VectorXDecisionVariable> state_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> force_vars_;
  std::vector<drake::solvers::VectorXDecisionVariable> stance_vars_;
  int n_c_;
} CentroidalMode;

class PlanarCentroidalTrajOpt : public drake::solvers::MathematicalProgram {
 public:
  /// Constructor
  PlanarCentroidalTrajOpt(double I, double mass, double h, double mu);

  /// Specifies a final pose (com position and angle) by adding a bounding box
  /// constraint to be within +/- eps of the final pose
  void SetFinalPose(Eigen::Vector2d com, double theta, double eps=0.05);

  /// Adds a constraint on the final velocity
  void SetFinalVel(Eigen::Vector2d v, double omega, double eps=0.01);

  /// Constraint on the final state (position and velocity in one function)
  void SetFinalState(Eigen::VectorXd state);


  void SetInitialPose(Eigen::Vector2d com, double theta);
  void SetInitialVel(Eigen::Vector2d v, double omega);
  void SetModeSequence(std::vector<stance> sequence, std::vector<double> times);
  void SetNominalStance(Eigen::Vector2d left, Eigen::Vector2d right);
  void SetMaxDeviationConstraint(Eigen::Vector2d max);
  void SetInitialStateGuess();
  void SetInitialForceGuess();
  drake::solvers::MathematicalProgramResult SolveProg(int iteration_limit);

 private:
  std::vector<Eigen::Vector2d> nominal_stance_;
  std::vector<stance> sequence_;
  std::vector<double> times_;
  std::vector<CentroidalMode> modes_;
  Eigen::VectorXd x0_;
  Eigen::VectorXd xf_;

  const double I_;
  const double mass_;
  const double h_;
  const double mu_;
};
}
}
}


