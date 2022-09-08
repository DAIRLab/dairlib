#pragma once
#include <array>

#include "geometry/convex_foothold.h"
#include "solvers/nonlinear_constraint.h"
#include "unsupported/Eigen/MatrixFunctions"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib::systems::controllers {

inline std::vector<std::vector<int>> cartesian_product(unsigned long range, int sets) {

  auto products = std::vector<std::vector<int>>();
  for (int i = 0; i < pow(range, sets); i++) {
    products.emplace_back(std::vector<int>(sets, 0));
  }
  auto counter = std::vector<int>(sets, 0); // array of zeroes
  for (auto &product : products) {
    product = counter;

    // counter increment and wrapping/carry over
    counter.back()++;
    for (size_t i = counter.size()-1; i != 0; i--) {
      if (counter[i] == range) {
        counter[i] = 0;
        counter[i-1]++;
      }
      else break;
    }
  }
  return products;
}

using drake::solvers::Binding;
using solvers::NonlinearConstraint;
using drake::solvers::QuadraticCost;
using drake::solvers::DecisionVariable;
using drake::solvers::LinearConstraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::BoundingBoxConstraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::LinearEqualityConstraint;

class AlipDynamicsConstraint : public NonlinearConstraint<drake::AutoDiffXd> {
 public:
  AlipDynamicsConstraint(double m, double H, double n);
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>>& x,
                          drake::VectorX<drake::AutoDiffXd>* y) const override;
  Eigen::Matrix4d Ad(double t);

  void set_m(double m) {m_ = m;}
  void set_H(double H) {H_ = H;}

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
  AlipMINLP(double m, double H) : m_(m), H_(H) {};
  AlipMINLP(const AlipMINLP& rhs)=default;
  AlipMINLP& operator=(const AlipMINLP& rhs)=default;

  // Problem setup
  void AddMode(int nk);
  void AddInputCost(double R);
  void SetMinimumStanceTime(double tmin) {tmin_ = tmin;};
  void AddTrackingCost(const std::vector<std::vector<Eigen::Vector4d>> &xd,
  const Eigen::Matrix4d &Q);
  void AddFootholds(const std::vector<geometry::ConvexFoothold>& footholds) {
    footholds_.insert(footholds_.end(), footholds.begin(), footholds.end());
  }


  void Build();
  std::vector<std::vector<Eigen::Vector4d>> MakeXdesTrajForVdes(
      const Eigen::Vector2d& vdes, double step_width, double Ts, double nk,
      int stance=1) const;
  std::vector<Eigen::Vector4d> MakeXdesTrajForCurrentStep(
      const Eigen::Vector2d& vdes, double t_current, double t_remain,
      double Ts, double step_width, int stance, double nk) const;

  // per solve updates
  void UpdateInitialGuess();
  void DeactivateInitialTimeConstraint();
  void ActivateInitialTimeConstraint(double t);
  void UpdateNextFootstepReachabilityConstraint(
      const geometry::ConvexFoothold& fixed_workspace,
      const geometry::ConvexFoothold& inflating_workspace);

  void UpdateInitialGuess(const Eigen::Vector3d &p0, const Eigen::Vector4d &x0);
  void UpdateTrackingCost(const std::vector<std::vector<Eigen::Vector4d>>& xd);
  void ChangeFootholds(const std::vector<geometry::ConvexFoothold>& footholds) {
    ClearFootholds();
    AddFootholds(footholds);
  }
  void UpdateNominalStanceTime(double t0, double Ts) {
    std::vector<double> T(nmodes_, Ts);
    T.front() = t0;
    td_ = T;
  }

  // Solving the problem
  void CalcOptimalFootstepPlan(
      const Eigen::Vector4d &x, const Eigen::Vector3d &p, bool warmstart=false);

  // Getting the solution
  Eigen::VectorXd GetTimingSolution() const;
  std::vector<Eigen::Vector3d> GetFootstepSolution() const;
  std::vector<std::vector<Eigen::Vector4d>> GetStateSolution() const;
  std::vector<std::vector<Eigen::VectorXd>> GetInputSolution() const;

  // getters and setters
  void set_m(double m) { m_ = m; }
  void set_H(double H) { H_ = H; }
  int nmodes() const {return nmodes_;}
  std::vector<int> nknots() const {return nknots_;}
  std::vector<std::vector<Eigen::Vector4d>> get_xd(){ return xd_;}


 private:

  // problem data:
  // TODO(@Brian-Acosta): Make these gains/parameters as appropriate
  double m_;
  double H_;
  int np_ = 3;
  int nx_ = 4;
  int nu_ = 1;
  int nmodes_ = 0;
  double R_ = 0;
  double tmin_ = 0;   // gainify
  double tmax_ = 0.5; // gainify
  double vmax_ = 5.0; // avg 2 m/s
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Qf_;
  std::vector<double> td_;
  std::vector<int> nknots_{};
  std::vector<std::vector<Eigen::Vector4d>> xd_;
  std::vector<std::vector<int>> mode_sequnces_{};
  std::vector<geometry::ConvexFoothold> footholds_;


  // problem setup methods
  void MakeTerminalCost();
  void MakeResetConstraints();
  void MakeDynamicsConstraints();
  void ClearFootholdConstraints();
  void MakeInitialTimeConstraint();
  void MakeTimingBoundsConstraint();
  void MakeInitialStateConstraint();
  void MakeInitialFootstepConstraint();
  void MakeNextFootstepReachabilityConstraint();
  std::vector<std::vector<int>> GetPossibleModeSequences();
  void MakeIndividualFootholdConstraint(int idx_mode, int idx_foothold);
  void MakeFootstepConstraints(std::vector<int> foothold_idxs);

  // per solve updates and solve steps
  void SolveOCProblemAsIs();
  void ClearFootholds() {footholds_.clear();}

  // program and decision variables
  drake::copyable_unique_ptr<MathematicalProgram>
      prog_{std::make_unique<MathematicalProgram>()};
  std::vector<VectorXDecisionVariable> pp_;
  std::vector<VectorXDecisionVariable> tt_;
  std::vector<std::vector<VectorXDecisionVariable>> xx_;
  std::vector<std::vector<VectorXDecisionVariable>> uu_;

  // constraints
  std::vector<Binding<BoundingBoxConstraint>> ts_bounds_c_{};
  std::vector<Binding<LinearEqualityConstraint>> reset_map_c_{};
  std::shared_ptr<LinearEqualityConstraint> initial_foot_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_time_c_ = nullptr;
  std::shared_ptr<LinearEqualityConstraint> initial_state_c_ = nullptr;
  std::shared_ptr<LinearConstraint> next_step_reach_c_fixed_ = nullptr;
  std::shared_ptr<LinearConstraint> next_step_reach_c_inflating_ = nullptr;
  std::shared_ptr<AlipDynamicsConstraint> dynamics_evaluator_ = nullptr;
  std::vector<std::vector<Binding<drake::solvers::Constraint>>> dynamics_c_;
  std::vector<std::pair<Binding<LinearConstraint>,
                        Binding<LinearEqualityConstraint>>> footstep_c_;

  // costs
  std::shared_ptr<QuadraticCost> terminal_cost_;
  std::vector<std::vector<Binding<QuadraticCost>>> input_costs_{};
  std::vector<std::vector<Binding<QuadraticCost>>> tracking_costs_{};

  // Bookkeeping
  bool built_ = false;
  std::vector<drake::solvers::MathematicalProgramResult> solutions_;



};
}
