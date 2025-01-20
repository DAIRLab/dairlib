#pragma once

#include "constrained_inverse_dynamics_info.h"
#include "knot_point_state.h"

namespace dairlib::systems::controllers::id_mpc {

struct knot_config {
  int index;
  bool state_only;
  bool include_torques;
  std::vector<int> active_constraint_indices;
  std::vector<int> active_constraint_dot_indices;
};

class KnotPoint {
 public:
  template<typename T>
  const drake::VectorX<T> get_q(const drake::VectorX<T>& full_vars) const;

  template<typename T>
  const drake::VectorX<T> get_v(const drake::VectorX<T>& full_vars) const;

  template<typename T>
  const drake::VectorX<T> get_x(const drake::VectorX<T>& full_vars) const;

  template<typename T>
  const drake::VectorX<T> get_lh(const drake::VectorX<T>& full_vars) const;

  template<typename T>
  const drake::VectorX<T> get_lc(const drake::VectorX<T>& full_vars) const;

  template<typename T>
  const drake::VectorX<T> get_lambda(const drake::VectorX<T>& full_vars) const;

  template<typename T>
  const drake::VectorX<T> get_u(const drake::VectorX<T>& full_vars) const;


  explicit KnotPoint(
      const ConstrainedDynamicsInfo& dynamics, knot_config config);

  /*!
   * whether the config for this nkot includes torques
   */
   bool has_torques() const {return config_.include_torques;}

  /*!
   * Index of this knot point in the trajectory
   * @return
   */
  [[nodiscard]] int index() const {
    return config_.index;
  }

  /*!
   * @return The size of the state variables (q, v) at this knot point
   */
  [[nodiscard]] size_t num_state_variables() const {
    return dynamics_info_.nx();
  }

  /*!
   * @return The size of the input variables ([u], lambda_h, lambda_c)
   * where brackets denote that u may not be present
   */
  [[nodiscard]] size_t num_input_variables() const {
    if (config_.state_only) {
      return 0;
    }
    if (config_.include_torques) {
      return dynamics_info_.nc() + dynamics_info_.nh() + dynamics_info_.nu();
    }
    return dynamics_info_.nc() + dynamics_info_.nh();
  }

  /*!
   * @return the total number of state/input decision variables associated with
   * this knot point
   */
  [[nodiscard]] size_t total_variables() const {
    return num_input_variables() + num_state_variables();
  }


  /*!
   * @return The number of rows in the stacked kinematic constraint, after
   * mapping to the active indices
   */
  [[nodiscard]] size_t kinematic_constraint_dimension() const {
    return config_.active_constraint_indices.size() +
           config_.active_constraint_dot_indices.size();
  };

  [[nodiscard]] size_t dynamics_constraint_dimension() const {
    return vdot_constraint_dimension() + dynamics_info_.nq();
  }

  [[nodiscard]] size_t vdot_constraint_dimension() const {
    return config_.include_torques ?
           dynamics_info_.nv() : dynamics_info_.nv() - dynamics_info_.nu();
  }

  template <typename T>
  drake::VectorX<T> EvalInverseDynamicsDefect(
      KnotPointState* cache,
      const drake::VectorX<T>& x,
      const drake::VectorX<T>& lambdas_and_maybe_us,
      const drake::VectorX<T>& vdot) const;

  template <typename T>
  drake::VectorX<T> EvalKinematicConstraints(
      KnotPointState* cache, const drake::VectorX<T>& x) const;

 private:
  const ConstrainedDynamicsInfo& dynamics_info_;
  knot_config config_;

  std::vector<int> unactuated_vdot_coords_;
  Eigen::MatrixXd actuation_matrix_;

  int nq_;
  int nv_;
  int nh_;
  int nc_;
  int nu_;

};

}