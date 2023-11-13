#pragma once
#include <array>
#include <chrono>

#include "alip_utils.h"
#include "geometry/convex_polygon.h"
#include "solvers/nonlinear_constraint.h"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/snopt_solver.h"

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

struct profile_data {
  std::chrono::time_point<std::chrono::steady_clock> start_;
  std::chrono::time_point<std::chrono::steady_clock> finish_;
  double solve_time_ = 0;
  friend std::ostream& operator<<(std::ostream& os, const profile_data& data);
};

inline std::ostream& operator<<(std::ostream& os, const profile_data& data) {
  os << "total: " << (data.finish_ - data.start_).count() << ", solver: " << data.solve_time_;
  return os;
}


/// Base class to handle boilerplate for Alip MPC controllers
/// Specific controllers should implement AddMode, Build, SolveOCProblemAsIs,
/// UpdateInitialGuess, and any required helper functions
class AlipMPC {
 public:

  template<typename Derived>
  static Eigen::VectorBlock<Derived, 4>
  GetStateAtKnot(Eigen::MatrixBase<Derived>& xx, int knot) {
    return Eigen::VectorBlock<Derived, 4>(xx.derived(), 4 * knot);
  }
  template<typename Derived>
  static Eigen::VectorBlock<const Derived, 4>
  GetStateAtKnot(const Eigen::MatrixBase<Derived>& xx, int knot) {
    return Eigen::VectorBlock<const Derived, 4>(xx.derived(), 4 * knot);
  }
  template<typename Derived>
  static Eigen::VectorBlock<Derived, 1>
  GetInputAtKnot(Eigen::MatrixBase<Derived>& uu, int knot) {
    return Eigen::VectorBlock<Derived, 1>(uu.derived(), knot);
  }
  template<typename Derived>
  static Eigen::VectorBlock<const Derived, 1>
  GetInputAtKnot(const Eigen::MatrixBase<Derived>& uu, int knot) {
    return Eigen::VectorBlock<const Derived, 1>(uu.derived(), knot);
  }

  /// Constructor takes a nominal robot mass and walking height
  AlipMPC(double m,
          double H,
          int nknots,
          alip_utils::ResetDiscretization reset_discretization)
      : m_(m), H_(H), nknots_(nknots),
        reset_discretization_(reset_discretization){};

  /* -- Problem setup - these must be called before Build() -- */
  virtual void AddMode() = 0;
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
      const vector<Eigen::VectorXd> &xd,
      const Eigen::Matrix4d &Q, const Eigen::MatrixXd &Qf);
  void AddFootholds(const vector<geometry::ConvexPolygon> &footholds) {
    footholds_.insert(footholds_.end(), footholds.begin(), footholds.end());
  }
  std::vector<geometry::ConvexPolygon> footholds() const {return footholds_;}
  void AddFootholdRegularization(const Eigen::MatrixXd& W_footstep_reg);
  void UpdateFootholdRegularization(double scale, const Eigen::Vector3d& pST_SW);

  virtual void Build(const drake::solvers::SolverOptions &solver_options);
  virtual void Build() = 0;

  vector<Eigen::VectorXd> MakeXdesTrajForVdes(
      const Eigen::Vector2d &vdes, double step_width, double Ts, int nk,
      alip_utils::Stance stance = alip_utils::Stance::kRight) const;

  // per solve updates
  void UpdateMaximumCurrentStanceTime(double tmax);
  void ActivateInitialTimeEqualityConstraint(double t);
  void UpdateNextFootstepReachabilityConstraint(
      const geometry::ConvexPolygon &workspace);
  void UpdateTrackingCost(const vector<Eigen::VectorXd>& xd);
  void UpdateFootholds(const vector<geometry::ConvexPolygon> &footholds) {
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
  Eigen::VectorXd GetTimingDesired() const;
  vector<Eigen::VectorXd> GetStateDesired() const { return xd_; }
  vector<Eigen::Vector3d> GetFootstepDesired() const {
    return vector<Eigen::Vector3d>(nmodes_, Eigen::Vector3d::Zero()); // NOLINT(modernize-return-braced-init-list)
  };
  vector<Eigen::VectorXd> GetInputDesired() const {
    return vector<Eigen::VectorXd>( // NOLINT(modernize-return-braced-init-list)
        nmodes_, Eigen::VectorXd::Zero(nu_ * (nknots_ - 1)));
  };

  // misc getters and setters
  double solve_time() const {
    return std::chrono::duration<double>(solve_time_.finish_ - solve_time_.start_).count();
  };

  double optimizer_time() const {
    return solve_time_.solve_time_;
  }

  void set_m(double m) { m_ = m; }
  void set_H(double H) { H_ = H; }
  void set_xlim(double xlim) {xlim_ = xlim;}
  void set_ylim(double ylim) {ylim_ = ylim;}
  double H() const {return H_;}
  double m() const {return m_;}
  int nmodes() const { return nmodes_; }
  int nknots() const { return nknots_; }

  drake::solvers::MathematicalProgram *get_prog() { return prog_.get_mutable(); }
  const drake::solvers::MathematicalProgramResult& get_solution() const { return solution_.first; }

  double GetTerminalCost() const;
  double GetRunningCost() const;
  double GetInputCost() const;
  double GetSoftConstraintCost() const;

 protected:

  // problem data:
  // TODO(@Brian-Acosta): Make these gains/parameters as appropriate
  double m_;
  double H_;
  const int nknots_;  // knots per mode
  const alip_utils::ResetDiscretization reset_discretization_;
  const int np_ = 3;
  const int nx_ = 4;
  const int nu_ = 1;
  int nmodes_ = 0;
  double Tds_ = 0;
  double R_ = 0;
  double umax_ = -1;
  double ylim_ = 0.35;
  double xlim_ = 0.4;
  vector<double> tmin_{};
  vector<double> tmax_{};
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Qf_;
  Eigen::MatrixXd W_footstep_reg_;
  vector<double> td_;
  vector<Eigen::VectorXd> xd_;
  vector<vector<int>> mode_sequnces_{};
  vector<geometry::ConvexPolygon> footholds_;
  profile_data solve_time_;

  // problem setup methods
  void MakeTerminalCost();
  void MakeResetConstraints();
  void MakeDynamicsConstraints();
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
  virtual void SolveOCProblemAsIs() = 0;
  virtual void UpdateInitialGuess() = 0;
  virtual void UpdateInitialGuess(const Eigen::Vector3d &p0, const Eigen::Vector4d &x0) = 0;
  void UpdateTimingGradientStep();
  void UpdateDynamicsConstraints();
  void ClearFootholds() { footholds_.clear(); }
  vector<Eigen::VectorXd>
      ExtractDynamicsConstraintDual(
          const drake::solvers::MathematicalProgramResult& sol);

  // program and decision variables
  drake::copyable_unique_ptr<MathematicalProgram>
      prog_{std::make_unique<MathematicalProgram>()};
  vector<VectorXDecisionVariable> pp_{};
  vector<VectorXDecisionVariable> xx_{};
  vector<VectorXDecisionVariable> uu_{};
  vector<VectorXDecisionVariable> ee_{};

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
  std::shared_ptr<QuadraticCost> footstep_position_regularization_ = nullptr;
  vector<Binding<QuadraticCost>> input_costs_{};
  vector<vector<Binding<QuadraticCost>>> soft_constraint_cost_{};
  vector<vector<Binding<QuadraticCost>>> tracking_costs_{};

  // Bookkeeping
  bool built_ = false;
  std::pair<drake::solvers::MathematicalProgramResult,
            vector<Eigen::VectorXd>> solution_;
};
}
