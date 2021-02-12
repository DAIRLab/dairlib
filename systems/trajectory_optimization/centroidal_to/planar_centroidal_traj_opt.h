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

#include "lcm/lcm_trajectory.h"


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
  drake::solvers::VectorXDecisionVariable stance_vars_;
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
  void SetFlatGroundTerrainConstraint();
   // void SetFootPlacementContinuityConstraint();
  void SetInitialStateGuess();
  void SetInitialForceGuess();
  void SetInitialStanceGuess();
  double MapKnotPointToTime(int idx_mode, int idx_knot);
  int NumStateKnots();
  LcmTrajectory GetStateTrajectories(drake::solvers::MathematicalProgramResult& result);
  LcmTrajectory GetFootTrajectories(drake::solvers::MathematicalProgramResult& result);
  LcmTrajectory GetForceTrajectories(drake::solvers::MathematicalProgramResult& result);
  drake::solvers::MathematicalProgramResult SolveProg(int iteration_limit);

  std::vector<CentroidalMode> modes() { return modes_ ;};
  std::vector<stance> sequence() { return sequence_;}

 private:
  std::vector<std::string> state_var_names_ =
      {"x", "y", "theta", "x_dot", "y_dot", "theta_dot"};
  std::vector<std::string> stance_var_names_ =
      {"left", "right"};
  std::vector<std::string> force_var_names_ =
      {"left_force", "right_force"};

  std::vector<Eigen::Vector2d> nominal_stance_;
  std::vector<stance> sequence_;
  std::vector<double> times_;
  std::vector<CentroidalMode> modes_;
  Eigen::VectorXd x0_;
  Eigen::VectorXd xf_;

  int n_modes_ = 0;
  const double I_;
  const double mass_;
  const double h_;
  const double mu_;
};
}
}
}


