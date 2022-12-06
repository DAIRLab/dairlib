#pragma once
#include <array>

#include "alip_utils.h"
#include "geometry/convex_foothold.h"
#include "solvers/nonlinear_constraint.h"
#include "unsupported/Eigen/MatrixFunctions"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/mosek_solver.h"

namespace dairlib::systems::controllers {

using solvers::NonlinearConstraint;

using drake::solvers::Binding;
using drake::solvers::OsqpSolver;
using drake::solvers::MosekSolver;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

using std::vector;

/*
 * Nonlinear constraint representing the linear ALIP dynamics with ankle torque,
 * discretized with stance duration t as a decision variable,
 * with n evenly-spaced knot points
 */
class AlipDynamicsConstraint : public NonlinearConstraint<drake::AutoDiffXd> {
 public:
  AlipDynamicsConstraint(double m, double H, double n);
  void EvaluateConstraint(
      const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>> &x,
      drake::VectorX<drake::AutoDiffXd> *y) const override;
  Eigen::Matrix4d Ad(double t);

  void set_m(double m) {
    m_ = m;
    A_ = alip_utils::CalcA(H_, m_);
    A_inv_ = A_.inverse();
  }
  void set_H(double H) {
    H_ = H;
    A_ = alip_utils::CalcA(H_, m_);
    A_inv_ = A_.inverse();
  }

 private:
  double m_;
  double H_;
  double n_;
  Eigen::Matrix4d A_;
  Eigen::Matrix4d A_inv_;
  Eigen::MatrixXd B_;
};

class AlipMINLP {
 public:

  template<typename T>
  static Eigen::Ref<drake::VectorX<T>> GetStateAtKnot(VectorX<T> xx, int knot) {
    return xx.segment<4>(4 * knot);
  }
  template<typename T>
  static Eigen::Ref<drake::VectorX<T>> GetInputAtKnot(VectorX<T> uu, int knot) {
    return uu.segment<1>(knot);
  }

  /// Constructor takes a nominal robot mass and walking height
  AlipMINLP(double m, double H) : m_(m), H_(H) {};

  /* -- Problem setup - these must be called before Build() -- */

  void AddMode(int n_knots);
  void AddInputCost(double R);
  void SetMinimumStanceTime(double tmin) {
    tmin_ = vector<double>(nmodes_, tmin);
  };
  void SetMaximumStanceTime(double tmax) {
    tmax_ = vector<double>(nmodes_, tmax);
  };
  void SetDoubleSupportTime(double t_ds) { Tds_ = t_ds; }
  void SetInputLimit(double umax) { umax_ = umax; };
  void AddTrackingCost(
      const vector<vector<Eigen::Vector4d>> &xd,
      const Eigen::Matrix4d &Q, const Eigen::MatrixXd &Qf
  );
  void AddFootholds(const vector<geometry::ConvexFoothold> &footholds) {
    footholds_.insert(footholds_.end(), footholds.begin(), footholds.end());
  }

  void Build(const drake::solvers::SolverOptions &solver_options);
  vector<vector<Eigen::Vector4d>> MakeXdesTrajForVdes(
      const Eigen::Vector2d &vdes, double step_width, double Ts, double nk,
      alip_utils::Stance stance = alip_utils::Stance::kRight) const;
  vector<Eigen::Vector4d> MakeXdesTrajForCurrentStep(
      const Eigen::Vector2d &vdes, double t_current, double t_remain,
      double Ts, double step_width, alip_utils::Stance stance, double nk) const;

  // per solve updates
  void UpdateMaximumCurrentStanceTime(double tmax);
  void ActivateInitialTimeEqualityConstraint(double t);
  void UpdateNextFootstepReachabilityConstraint(
      const geometry::ConvexFoothold &workspace);
  void UpdateTrackingCost(const vector<vector<Eigen::Vector4d>> &xd);
  void UpdateFootholds(const vector<geometry::ConvexFoothold> &footholds) {
    ClearFootholds();
    AddFootholds(footholds);
  }
  void UpdateNominalStanceTime(double t0, double Ts) {
    vector<double> T(nmodes_, Ts);
    T.front() = t0;
    td_ = T;
  }

  // Update the dynamics constraint to reflect the current mode timings
  // after maybe updating the mode timing by one gradient step
  void UpdateModeTiming(bool take_sqp_step);
  void UpdateNoCrossoverConstraint();
  void UpdateModeTimingsOnTouchdown();

  // Solving the problem
  void CalcOptimalFootstepPlan(
      const Eigen::Vector4d &x,
      const Eigen::Vector3d &p,
      bool warmstart = false);

  // Getting the solution
  Eigen::VectorXd GetTimingSolution() const;
  vector<Eigen::Vector3d> GetFootstepSolution() const;
  vector<Eigen::VectorXd> GetStateSolution() const;
  vector<Eigen::VectorXd> GetInputSolution() const;

  // getting the initial guess
  Eigen::VectorXd GetTimingGuess() const;
  vector<Eigen::Vector3d> GetFootstepGuess() const;
  vector<Eigen::VectorXd> GetStateGuess() const;
  vector<Eigen::VectorXd> GetInputGuess() const;

  // getting the desired solution
  Eigen::VectorXd GetDesiredTiming() const;
  vector<Eigen::VectorXd> GetDesiredState() const {
    return xd_;
  }
  vector<Eigen::Vector3d> GetDesiredFootsteps() const {
    return vector<Eigen::Vector3d>(nmodes_, Eigen::Vector3d::Zero()); // NOLINT(modernize-return-braced-init-list)
  };
  vector<Eigen::VectorXd> GetDesiredInputs() const {
    return vector<Eigen::VectorXd>( // NOLINT(modernize-return-braced-init-list)
        nmodes_, Eigen::VectorXd::Zero(nu_ * (nknots_.front() - 1)));
  };

  // misc getters and setters
  double solve_time() const;
  void set_m(double m) { m_ = m; }
  void set_H(double H) { H_ = H; }
  double H() const {return H_;}
  double m() const {return m_;}
  int nmodes() const { return nmodes_; }
  vector<int> nknots() const { return nknots_; }

  drake::solvers::MathematicalProgram *get_prog() { return prog_.get_mutable(); }
  drake::solvers::MathematicalProgramResult &get_solution() { return solution_.first; }

 private:

  // problem data:
  // TODO(@Brian-Acosta): Make these gains/parameters as appropriate
  double m_;
  double H_;
  const int np_ = 3;
  const int nx_ = 4;
  const int nu_ = 1;
  int nmodes_ = 0;
  double Tds_ = 0;
  double R_ = 0;
  double umax_ = -1;
  vector<double> tmin_{};
  vector<double> tmax_{};
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Qf_;
  vector<double> td_;
  vector<int> nknots_{};
  vector<Eigen::VectorXd> xd_;
  vector<vector<int>> mode_sequnces_{};
  vector<geometry::ConvexFoothold> footholds_;

  // problem setup methods
  void MakeTerminalCost();
  void MakeResetConstraints();
  void MakeDynamicsConstraints();
  void MakeFootholdConstraints();
  void MakeWorkspaceConstraints();
  void MakeInputBoundConstaints();
  void MakeNoCrossoverConstraint();
  void MakeInitialStateConstraint();
  void MakeInitialFootstepConstraint();
  void MakeNextFootstepReachabilityConstraint();
  void MakeCapturePointConstraint(int foothold_idx);

  void CalcResetMap(Eigen::Matrix<double,4,12>* Aeq) const;
  vector<vector<int>> GetPossibleModeSequences();

  // per solve updates and solve steps
  void SolveOCProblemAsIs();
  void UpdateInitialGuess();
  void UpdateInitialGuess(const Eigen::Vector3d &p0, const Eigen::Vector4d &x0);
  void UpdateTimingGradientStep();
  void UpdateDynamicsConstraints();
  void ClearFootholds() { footholds_.clear(); }

  // Per QP updates
  void UpdateFootholdConstraints(vector<int> foothold_idxs);
  void UpdateIndividualFootholdConstraint(int idx_mode, int idx_foothold);

  // program and decision variables
  drake::copyable_unique_ptr<MathematicalProgram>
      prog_{std::make_unique<MathematicalProgram>()};
  vector<VectorXDecisionVariable> pp_{};
  vector<VectorXDecisionVariable> xx_{};
  vector<VectorXDecisionVariable> uu_{};

  // store the mode timing as doubles and take gradient steps external to the QP
  Eigen::VectorXd tt_;

  // constraints
  vector<Binding<BoundingBoxConstraint>> input_bounds_c_{};
  vector<Binding<LinearEqualityConstraint>> reset_map_c_{};
  vector<Binding<LinearConstraint>> no_crossover_constraint_{};
  std::shared_ptr<LinearEqualityConstraint> initial_foot_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_state_c_ = nullptr;
  std::shared_ptr<LinearConstraint> next_step_reach_c_fixed_ = nullptr;
  std::shared_ptr<Binding<LinearConstraint>> capture_point_contstraint_ = nullptr;
  vector<vector<Binding<LinearEqualityConstraint>>> dynamics_c_{};
  vector<std::pair<Binding<LinearConstraint>,
                   Binding<LinearEqualityConstraint>>> footstep_c_{};

  // costs
  std::shared_ptr<QuadraticCost> terminal_cost_;
  vector<Binding<QuadraticCost>> input_costs_{};
  vector<vector<Binding<QuadraticCost>>> tracking_costs_{};

  // Bookkeeping
  bool built_ = false;
  std::pair<drake::solvers::MathematicalProgramResult, vector<Eigen::VectorXd>> solution_;

  drake::solvers::OsqpSolver solver_;

};
}
