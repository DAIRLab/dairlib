#include "dairlib/lcmt_radio_out.hpp"
#include "hip_yaw_traj_gen.h"

using Eigen::VectorXd;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib::cassie {

HipYawTrajGen::HipYawTrajGen(int left_stance_state) :
 left_stance_state_(left_stance_state) {
  fsm_port_ = this->DeclareVectorInputPort(
      "fsm", drake::systems::BasicVector<double>(1))
          .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("radio",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;

  hip_yaw_traj_port_ = this->DeclareAbstractOutputPort(
          "hip_yaw_traj", traj_inst, &HipYawTrajGen::CalcYawTraj)
      .get_index();
}

void HipYawTrajGen::CalcYawTraj(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

  VectorXd yaw = VectorXd::Ones(1);
  auto radio = this->EvalAbstractInput(
      context, radio_port_)->get_value<lcmt_radio_out>();
  int fsm = this->EvalVectorInput(context, fsm_port_)->value()(0);

  double sign = (fsm == left_stance_state_) ? -1.0 : 1.0;
  /// TODO (@Brian-Acosta) add constructor arg for joint limit
  yaw(0) = sign * 0.393 * radio.channel[7];
  const auto pp = PiecewisePolynomial<double>(yaw);
  // Assign traj
  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = pp;
}
}