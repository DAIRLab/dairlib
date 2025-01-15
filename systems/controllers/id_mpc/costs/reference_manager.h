#pragma once

#include <string>
#include <unordered_map>

#include "nonlinear_least_squares_cost.h"
#include "quadratic_error_cost.h"

#include "drake/common/trajectories/piecewise_polynomial.h"


namespace dairlib::systems::controllers::id_mpc {


/*!
 * The reference manager is responsible for holding and updating the evaluators
 * for costs on different reference trajectories
 */
template <typename T>
class ReferenceManager {
 public:
  ReferenceManager() = default;
  ReferenceManager(int N, double dt): N_(N), dt_(dt){};

  /*!
   * Construct N evaluators for a specific cost type.
   * @tparam C type of cost to add
   * @param name name of the cost
   * @param args arguments to c's constructor
   */
  template<template <typename S> class C, typename... Args>
  void AddRunningStateCost(const std::string& name, Args&&... args) {
    static_assert(
        std::derived_from<C<T>, NonlinearLeastSquaresCost<T>> == true);

    DRAKE_DEMAND(not evaluators_.contains(name));
    std::vector<std::shared_ptr<NonlinearLeastSquaresCost<T>>> costs;

    for (int i = 0; i < N_ + 1; ++i) {
      // trapezoidal integration
      double s = (i == 0 or i == N_) ? 0.5 : 1.0;
      auto ptr = std::make_shared<C<T>>(std::forward(args)...);
      costs.push_back(std::move(ptr));
      costs.back().MultiplyByScalar(s * dt_);
    }
    evaluators_[name] = costs;
  }

  template<typename... Args>
  void AddRunningInputCost(const std::string& name, Args&&... args) {
    DRAKE_DEMAND(not evaluators_.contains(name));
    std::vector<std::shared_ptr<QuadraticErrorCost<T>>> costs;
    for (int i = 0; i < N_; ++i) {
      auto ptr = std::make_shared<QuadraticErrorCost<T>>(std::forward(args)...);
      ptr->MultiplyByScalar(dt_);
      costs.push_back(ptr);
    }
    evaluators_[name] = costs;
  }

  void UpdateReference(
      const std::string& name,
      const drake::trajectories::Trajectory<double>& traj,
      const std::vector<double>& breaks);

  std::shared_ptr<NonlinearLeastSquaresCost<T>> GetEvaluator(
      const std::string& name, int i) {
    DRAKE_DEMAND(i <= N_);
    return evaluators_.at(name).at(i);
  }
 private:
  std::unordered_map<
      std::string, std::vector<std::shared_ptr<NonlinearLeastSquaresCost<T>>>> evaluators_;

  int N_;
  double dt_;
};

}