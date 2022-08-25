#pragma once
#include <array>

#include "geometry/convex_foothold.h"
#include "solvers/nonlinear_constraint.h"
#include "unsupported/Eigen/MatrixFunctions"

#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib::systems::controllers {

std::vector<std::vector<int>> cartesian_product(unsigned long range, int sets) {

  auto products = std::vector<std::vector<int>>();
  for (int i = 0; i < pow(range, sets); i++) {
    products.emplace_back(std::vector<int>(range, 0));
  }
  auto counter = std::vector<int>(range, 0); // array of zeroes
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
  AlipDynamicsConstraint(double m, double H);
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<drake::AutoDiffXd>>& x,
                          drake::VectorX<drake::AutoDiffXd>* y) const;
 private:
  double m_;
  double H_;
  Eigen::Matrix4d A_;
  Eigen::Matrix4d A_inv_;
  Eigen::MatrixXd B_;
};

class AlipMINLP {
 public:

  AlipMINLP(double m, double H) : m_(m), H_(H) {}

  // Problem setup
  void AddMode(int nk);
  void AddTrackingCost(std::vector<std::vector<Eigen::Vector4d>> xd,
                       Eigen::Matrix4d Q);
  void AddInputCost(double R);
  void Build();
  std::vector<std::vector<Eigen::Vector4d>> MakeXdesTrajForVdes(
      double vdes, double step_width, double Ts, double nk) const;

  // Solving the problem
  void CalcOptimalFoostepPlan(
      const Eigen::Vector4d &x, const Eigen::Vector3d &p);

  // Getting the solution
  std::vector<Eigen::Vector3d> GetFootstepSolution() const;
  std::vector<std::vector<Eigen::Vector4d>> GetStateSolution() const;
  std::vector<std::vector<Eigen::VectorXd>> GetInputSolution() const;
  Eigen::VectorXd GetTimingSolution() const;

  void set_m(double m) { m_ = m; }
  void set_H(double H) { H_ = H; }

 private:

  int np_ = 3;
  int nx_ = 4;
  int nu_ = 1;
  int nmodes_ = 0;
  std::vector<int> nknots_{};
  double m_;
  double H_;

  void SolveOCProblemAsIs();
  void ClearFootholdConstraints();
  void MakeIndividualFootholdConstraint(int idx_mode, int idx_foothold);
  void MakeFootstepConstraints(std::vector<int> foothold_idxs);
  void MakeResetConstraints();
  void MakeDynamicsConstraints();
  void MakeTimingBoundsConstraint();
  void MakeInitialStateConstraint();
  void MakeInitialFootstepConstraint();
  std::vector<std::vector<int>> GetPossibleModeSequences();

  std::vector<geometry::ConvexFoothold> footholds_;
  MathematicalProgram prog_ = MathematicalProgram{};
  std::vector<VectorXDecisionVariable> pp_;
  std::vector<std::vector<VectorXDecisionVariable>> xx_;
  std::vector<std::vector<VectorXDecisionVariable>> uu_;
  std::vector<VectorXDecisionVariable> tt_;

  std::vector<std::vector<Binding<drake::solvers::Constraint>>> dynamics_c_;
  std::vector<std::pair<Binding<LinearConstraint>, Binding<LinearEqualityConstraint>>> footstep_c_;
  std::vector<Binding<LinearEqualityConstraint>> reset_map_c_;
  std::vector<Binding<BoundingBoxConstraint>> ts_bounds_c_;
  LinearEqualityConstraint *initial_state_c_;
  LinearEqualityConstraint *initial_foot_c_;

  std::vector<std::vector<Binding<QuadraticCost>>> tracking_costs_;
  std::vector<std::vector<Binding<QuadraticCost>>> input_costs_;

  std::vector<drake::solvers::MathematicalProgramResult> solutions_;
  std::vector<std::vector<int>> mode_sequnces_{};

};
}
