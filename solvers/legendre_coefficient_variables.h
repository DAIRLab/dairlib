#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib {
namespace solvers {

class LegendreCoefficientVariables {
 public:
  LegendreCoefficientVariables(drake::solvers::MathematicalProgram& prog,
                                int degree, std::string name);

  void AddDerivativeConstraint(
      drake::solvers::MathematicalProgram& prog,
      std::string name, double time, int deriv_order, double value);

  void UpdateDerivativeConstraint(
      std::string name, double time, int deriv_order, double value);

  void AddDerivativeSoftConstraint(
      drake::solvers::MathematicalProgram& prog,
      std::string name, double time, int deriv_order, double value,
      drake::solvers::VectorXDecisionVariable slack_var);

  void UpdateDerivativeSoftConstraint(
      std::string name, double time, int deriv_order, double value);

  void AddRunningCostOnDerivativeSquared(
      drake::solvers::MathematicalProgram& prog,
      std::string name, int derivative_order, double scale=1.0);

  Eigen::VectorXd GetSolution(
      const drake::solvers::MathematicalProgramResult& result) const {
    return result.GetSolution(c_);
  }

 private:
  int degree_;
  drake::solvers::VectorXDecisionVariable c_;
  std::map<std::string, std::shared_ptr<drake::solvers::QuadraticCost>> my_costs_;
  std::map<std::pair<int, std::string>,
           std::shared_ptr<drake::solvers::LinearEqualityConstraint>> my_equality_constraints_;
};

}
}