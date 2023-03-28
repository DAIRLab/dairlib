#include "c3_controller.h"

#include <iostream>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib {

using drake::systems::BasicVector;
using Eigen::VectorXd;
using solvers::LCS;
using solvers::C3MIQP;

namespace systems {

C3Controller::C3Controller(LCS& lcs, C3Options c3_options) : lcs_(lcs) {
  target_input_port_ =
      this->DeclareVectorInputPort("desired_position",
                                   BasicVector<double>(2 + 16))
          .get_index();
  lcs_state_input_port_ =
      this->DeclareVectorInputPort(
              "x_lcs", BasicVector<double>(10))
          .get_index();

  std::vector<VectorXd> x_desired = {VectorXd::Zero(3)};
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
  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const BasicVector<double>& x =
      *this->template EvalVectorInput<BasicVector>(context, lcs_state_input_port_);
  // delta
  // in order:
  // state, forces, inputs
  // initialize with zeros or current state
  std::vector<VectorXd> delta(1, VectorXd::Zero(1));
  std::vector<VectorXd> w(1, VectorXd::Zero(1));
  c3_->Solve(x.value(), delta, w);
}

}  // namespace systems
}  // namespace dairlib
