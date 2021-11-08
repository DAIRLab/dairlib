#pragma once

#include <tuple>
#include <vector>
#include <memory.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"

namespace dairlib::systems {

class LipmMpc : public drake::solvers::MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LipmMpc)

  LipmMpc(const std::vector<Eigen::Vector2d>& des_xy_pos,
          const std::vector<Eigen::Vector2d>& des_xy_vel,
          double w_predict_lipm_p, double w_predict_lipm_v,
          const Eigen::VectorXd& init_pos, const Eigen::VectorXd& init_vel,
          const Eigen::VectorXd& init_input, int n_step,
          double first_mode_duration, double stride_period,
          std::vector<double> height_vec, double max_length_foot_to_body,
          double max_length_foot_to_body_front, double min_step_width,
          bool start_with_left_stance);
  ~LipmMpc() override {}

  /// Returns a vector of matrices containing the state values at
  /// each breakpoint at the solution for each mode of the trajectory.
  Eigen::MatrixXd GetStateSamples(
      const drake::solvers::MathematicalProgramResult& result) const;
  Eigen::MatrixXd GetInputSamples(
      const drake::solvers::MathematicalProgramResult& result) const;

  const drake::solvers::VectorXDecisionVariable& x_lipm_vars() const {
    return x_lipm_vars_;
  };

  const drake::solvers::VectorXDecisionVariable& u_lipm_vars() const {
    return u_lipm_vars_;
  };
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  x_lipm_vars_by_idx(int idx) const;
  const Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>
  u_lipm_vars_by_idx(int idx) const;

  void ConstructDynamicMatrices(double height, double stride_period,
                                Eigen::Matrix<double, 2, 2>* A,
                                Eigen::Matrix<double, 2, 1>* B);

  static Eigen::VectorXd GetStateSolutionByIndex(int idx, Eigen::VectorXd states) {
    return states.segment<4>(idx*4);
  }
  static Eigen::VectorXd GetInputSolutionByIndex(int idx, Eigen::VectorXd states) {
    return states.segment<2>(idx*2);
  }


  // Cost bindings
  std::vector<drake::solvers::Binding<drake::solvers::Cost>> lipm_p_bindings_;
  std::vector<drake::solvers::Binding<drake::solvers::Cost>> lipm_v_bindings_;

  // Decision variables (lipm state and input (foot location))
  drake::solvers::VectorXDecisionVariable x_lipm_vars_;
  drake::solvers::VectorXDecisionVariable u_lipm_vars_;

  int n_step_;
};
}  // namespace dairlib::systems
