#pragma once

#include "geometry/convex_foothold.h"
#include "solvers/nonlinear_constraint.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/decision_variable.h"

namespace dairlib::systems::controllers {

class AlipDynamicsConstraint : solvers::NonlinearConstraint<drake::AutoDiffXd> {
 public:
  AlipDynamicsConstraint();
  void EvaluateConstraint(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                          drake::AutoDiffVecXd* y) const;
};

class AlipMINLP {
 public:

  AlipMINLP(double m, double H) : m_(m), H_(H){}

  // Problem setup
  void AddMode(int nk);
  void AddTrackingCost(std::vector<std::vector<Eigen::Vector4d>> xd,
                       Eigen::Matrix4d Q);
  void AddInputCost(double R);
  void Build();
  std::vector<std::vector<Eigen::Vector4d>> MakeXdesTrajeForVdes(
      double vdes, double step_width, double Ts, double nk) const;

  // Solving the problem
  void CalcOptimalFoostepPlan(
      const Eigen::Vector4d& x, const Eigen::Vector3d& p);

  // Getting the solution
  std::vector<Eigen::Vector3d> GetFootstepSolution() const;
  std::vector<std::vector<Eigen::Vector4d>> GetStateSolution() const;
  std::vector<std::vector<double>> GetInputSolution() const;
  Eigen::VectorXd GetTimingSolution() const;


  void set_m(double m) {m_ = m;}
  void set_H(double H) {H_ = H;}

 private:
  double m_;
  double H_;
  void SolveOCProblemAsIs();
  void ClearFootholdConstraints();
  void MakeIndividualFootholdConstraint(int idx_mode, int idx_foothold);
  void MakeFootstepConstraints(std::vector<geometry::ConvexFoothold> footholds);
  void MakeResetConstraints();
  void MakeDynamicsConstraints();
  void MakeTimingBoundsConstraint();
  void MakeInitialStateConstraint();
  void MakeInitialFootstepConstraint();

  std::vector<geometry::ConvexFoothold> footholds_;
  drake::solvers::MathematicalProgram prog_ = drake::solvers::MathematicalProgram{};
  std::vector<drake::solvers::VectorXDecisionVariable> pp_;
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>> xx_;
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>> uu_;
  std::vector<drake::solvers::DecisionVariable> tt_;

  std::vector<std::vector<
  drake::solvers::Binding<drake::solvers::LinearEqualityConstraint>>> dynamics_c_;

  std::vector<std::pair<
  drake::solvers::Binding<drake::solvers::LinearConstraint>,
  drake::solvers::Binding<drake::solvers::LinearEqualityConstraint>>> footstep_c_;

  std::vector<drake::solvers::Binding<drake::solvers::LinearEqualityConstraint>> reset_map_c;
  std::vector<drake::solvers::Binding<drake::solvers::BoundingBoxConstraint>> ts_boudns_c_;
  drake::solvers::Binding<drake::solvers::LinearEqualityConstraint> initial_state_c_;
  drake::solvers::Binding<drake::solvers::LinearEqualityConstraint> initial_foot_c_;
};

}
