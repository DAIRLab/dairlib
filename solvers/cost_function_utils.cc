#include "cost_function_utils.h"
namespace dairlib {

namespace solvers {

using Eigen::VectorXd;
using drake::AutoDiffXd;
using drake::VectorX;
using drake::solvers::VectorXDecisionVariable;

template <typename T>
void AddPositiveWorkCost(
    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
    const drake::multibody::MultibodyPlant<T>& plant,
    double W) {
  for (int mode = 0; mode < trajopt.num_modes(); ++mode) {
    int mode_start = trajopt.get_mode_start(mode);
    int mode_end = trajopt.get_mode_start(mode) + trajopt.mode_length(mode) - 1;

    for (int i = mode_start; i < mode_end; ++i) {
      int n_vars = 1 + 2 * plant.num_velocities() + 2 * plant.num_actuators();
      int n_v = plant.num_velocities();
      int n_u = plant.num_actuators();
      drake::solvers::VectorXDecisionVariable variables(n_vars);
      variables(0) = trajopt.timestep(i)(0);
      variables.segment(1, n_v) =
          trajopt.state_vars(mode, i - mode_start).tail(n_v);
      variables.segment(1 + n_v, n_v) =
          trajopt.state_vars(mode, i + 1 - mode_start).tail(n_v);
      variables.segment(1 + 2 * n_v, n_u) =
          trajopt.input_vars(mode, i - mode_start);
      variables.segment(1 + 2 * n_v + n_u, n_u) =
          trajopt.input_vars(mode, i + 1 - mode_start);
      trajopt.AddCost(
          std::make_shared<PositiveMechanicalWork<T>>(n_v, plant.num_actuators(),
                                 plant.MakeActuationMatrix(), W,
                                 "pos_work_cost_" + std::to_string(i)),
          variables);
    }
  }
}

template <typename T>
PositiveMechanicalWork<T>::PositiveMechanicalWork(
    int n_v, int n_u, const Eigen::MatrixXd& B, const double W,
    const std::string& description)
    : solvers::NonlinearCost<T>((1 + 2 * n_v + 2 * n_u), description),
      W_(W),
      B_t_(B.transpose()),
      n_v_(n_v),
      n_u_(n_u) {
  // Checks for the provided decision vars
  DRAKE_DEMAND(B.rows() == n_v_);
  DRAKE_DEMAND(B.cols() == n_u_);
}

template <typename T>
T sigmoid(T val) {
  return exp(val) / (1 + exp(val));
}

template <typename T>
void PositiveMechanicalWork<T>::EvaluateCost(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  int start_idx = 0;
  auto h = x.segment(start_idx, 1);
  start_idx += 1;
  auto v_i = x.segment(start_idx, n_v_);
  start_idx += n_v_;
  auto v_ip1 = x.segment(start_idx, n_v_);
  start_idx += n_v_;
  auto u_i = x.segment(start_idx, n_u_);
  start_idx += n_u_;
  auto u_ip1 = x.segment(start_idx, n_u_);

  T cost = T();
  VectorX<T> actuated_velocities_i = B_t_ * v_i;
  VectorX<T> actuated_velocities_ip1 = B_t_ * v_ip1;
  T zero = T();
  for (int joint_idx = 0; joint_idx < n_u_; ++joint_idx) {
    cost += std::max(actuated_velocities_i(joint_idx) * u_i(joint_idx), zero);
    cost += std::max(actuated_velocities_ip1(joint_idx) * u_ip1(joint_idx), zero);
    // cost += sigmoid(actuated_velocities_i(joint_idx) * u_i(joint_idx));
    // cost += sigmoid(actuated_velocities_ip1(joint_idx) *u_ip1(joint_idx));
  }
  cost = 0.5 * h(0) * cost;
  cost *= W_;
  VectorX<T> cost_vec(1);
  cost_vec << cost;
  *y = cost_vec;
}

template void AddPositiveWorkCost(
    dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
    const drake::multibody::MultibodyPlant<double>& plant, double W);
//template void AddPositiveWorkCost(
//    dairlib::systems::trajectory_optimization::Dircon<AutoDiffXd>& trajopt,
//    const drake::multibody::MultibodyPlant<AutoDiffXd>& plant);

}  // namespace solvers
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::solvers::PositiveMechanicalWork)