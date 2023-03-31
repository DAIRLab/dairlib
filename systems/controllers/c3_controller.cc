#include "c3_controller.h"

#include <iostream>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib {

using drake::systems::BasicVector;
using Eigen::VectorXd;
using solvers::C3MIQP;
using solvers::LCS;
using systems::TimestampedVector;

namespace systems {

C3Controller::C3Controller(LCS& lcs, C3Options c3_options,
                           std::vector<Eigen::MatrixXd>& Q,
                           std::vector<Eigen::MatrixXd>& R,
                           std::vector<Eigen::MatrixXd>& G,
                           std::vector<Eigen::MatrixXd>& U)
    : lcs_(lcs), Q_(Q), R_(R), G_(G), U_(U), N_(Q_.size()) {
  DRAKE_DEMAND(Q_[0].rows() == lcs.A_[0].rows());
  target_input_port_ = this->DeclareVectorInputPort("desired_position",
                                                    BasicVector<double>(2 + 16))
                           .get_index();
  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs", TimestampedVector<double>(lcs.A_[0].rows()))
          .get_index();

  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(Q_.size() + 1, VectorXd::Zero(lcs.A_[0].rows()));
  c3_ = std::make_unique<C3MIQP>(lcs, Q_, R_, G_, U_, x_desired, c3_options);

  this->set_name("c3_controller");
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort("c3_output",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &C3Controller::OutputTrajectory)
          .get_index();
}

void C3Controller::OutputTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
//  const BasicVector<double>& x_des =
//      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const BasicVector<double>& x = *this->template EvalVectorInput<TimestampedVector>(
      context, lcs_state_input_port_);
  // delta
  // in order:
  // state, forces, inputs
  // initialize with zeros or current state
  std::vector<VectorXd> delta(N_, VectorXd::Zero(1));
  std::vector<VectorXd> w(N_, VectorXd::Zero(1));
  c3_->Solve(x.value(), delta, w);
}

}  // namespace systems
}  // namespace dairlib
