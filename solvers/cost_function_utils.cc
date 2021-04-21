#include "cost_function_utils.h"
namespace dairlib {

namespace solvers {

using drake::VectorX;
using drake::solvers::VectorXDecisionVariable;

template <typename T>
void AddPositiveWorkCost(
    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
    drake::multibody::MultibodyPlant<T>& plant) {
  double W = 0;
  for (int mode = 0; mode < trajopt.num_modes(); ++mode) {
    int mode_start = trajopt.get_mode_start(mode);
    int mode_end = trajopt.get_mode_start(mode) + trajopt.mode_length(mode);

    for (int i = mode_start; i < mode_end; ++i){
      trajopt->AddCost(
          PositiveMechanicalWork(trajopt.h_vars()(i - 1) + trajopt.h_vars()(i) / 2),
          trajopt.state(i), trajopt.input_vars(mode, i),
          plant.MakeActuationMatrix(), W, "pos_work_cost_" + std::to_string(i));
    }
  }
}

template <typename T>
PositiveMechanicalWork<T>::PositiveMechanicalWork(
    const Eigen::Ref<const VectorXDecisionVariable>& h_i,
    const Eigen::Ref<const VectorXDecisionVariable>& h_ip1,
    const Eigen::Ref<const VectorXDecisionVariable>& v_i,
    const Eigen::Ref<const VectorXDecisionVariable>& v_ip1,
    const Eigen::Ref<const VectorXDecisionVariable>& u_i,
    const Eigen::Ref<const VectorXDecisionVariable>& u_ip1,
    const Eigen::MatrixXd& B, const double W, const std::string& description)
    : solvers::NonlinearCost<T>((v_i.size() + u_i.size()), description),
      W_(W),
      B_t_(B.transpose()),
      n_v_(v_i.size()),
      n_u_(u_i.size()) {
  // Checks for the provided decision vars
  DRAKE_DEMAND(h_i.size() == 2);
  DRAKE_DEMAND(B.cols() == n_v_);
  DRAKE_DEMAND(B.rows() == n_u_);
}

template <typename T>
T sigmoid(T val) {
  return exp(val) / (1 + exp(val));
}

template <typename T>
void PositiveMechanicalWork<T>::EvaluateCost(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  int start_idx = 0;
  auto h_i = x.segment(start_idx, 1);
  start_idx += 1;
  auto h_ip1 = x.segment(start_idx, 1);
  start_idx += 1;
  auto v_i = x.segment(start_idx, n_v_);
  start_idx += n_v_;
  auto v_ip1 = x.segment(start_idx, n_v_);
  start_idx += n_v_;
  auto u_i = x.segment(start_idx, n_u_);
  start_idx += n_u_;
  auto u_ip1 = x.segment(start_idx, n_u_);
  auto step_size = h_ip1 - h_i;
  VectorX<T> cost = 0;
  VectorX<T> actuated_velocities_i = B_t_ * v_i;
  VectorX<T> actuated_velocities_ip1 = B_t_ * v_ip1;
  for (int joint_idx = 0; joint_idx < n_u_; ++joint_idx) {
    cost += std::max(actuated_velocities_i(joint_idx) * u_i(joint_idx), 0);
    cost += std::max(actuated_velocities_ip1(joint_idx) * u_ip1(joint_idx), 0);
    //cost += sigmoid(actuated_velocities_i(joint_idx) * u_i(joint_idx));
    //cost += sigmoid(actuated_velocities_ip1(joint_idx) *u_ip1(joint_idx));
  }
  cost *= 0.5 * step_size;
  cost *= W_;
  *y = cost;
}

}  // namespace solvers
}  // namespace dairlib