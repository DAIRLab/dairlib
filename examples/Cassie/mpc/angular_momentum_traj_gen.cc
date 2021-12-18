#include "angular_momentum_traj_gen.h"
#include "systems/framework/output_vector.h"
#include "lcm/lcm_trajectory.h"
#include "dairlib/lcmt_saved_traj.hpp"

#include "drake/common/trajectories/piecewise_polynomial.h"

using dairlib::multibody::SingleRigidBodyPlant;
using dairlib::LcmTrajectory;
using dairlib::systems::OutputVector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::math::RollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::trajectories::Trajectory;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib::mpc {

AngularMomentumTrajGen::AngularMomentumTrajGen(const Matrix3d& I) : I_b_(I) {
  mpc_traj_in_port_ = this->DeclareAbstractInputPort(
      "lcmt_saved_trajectory",
      drake::Value<dairlib::lcmt_saved_traj>{})
      .get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  traj_out_port_ = this->DeclareAbstractOutputPort(
      "orientation_traj", traj_inst, &AngularMomentumTrajGen::CalcTraj)
  .get_index();
}

void AngularMomentumTrajGen::CalcTraj(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  const auto& input_msg = input->get_value<lcmt_saved_traj>();

  LcmTrajectory lcm_traj(input_msg);
  LcmTrajectory::Trajectory orientation =
      lcm_traj.GetTrajectory("orientation");
  MatrixXd knots_dot =
      orientation.datapoints.block(3, 0, 3, orientation.datapoints.cols());

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
      orientation.time_vector, I_b_ * knots_dot);
}

}

