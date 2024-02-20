#include "end_effector_trajectory.h"

#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorTrajectoryGenerator::EndEffectorTrajectoryGenerator() {
  // Input/Output Setup
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(neutral_pose_);
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("end_effector_trajectory", traj_inst,
                                  &EndEffectorTrajectoryGenerator::CalcTraj);
}

void EndEffectorTrajectoryGenerator::SetRemoteControlParameters(
    const Eigen::Vector3d& neutral_pose, double x_scale, double y_scale,
    double z_scale) {
  neutral_pose_ = neutral_pose;
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void EndEffectorTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  const auto& trajectory_input =
      this->EvalAbstractInput(context, trajectory_port_)
          ->get_value<drake::trajectories::Trajectory<double>>();
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (radio_out->value()[14]) {
    PiecewisePolynomial<double> result;
    VectorXd y_0 = neutral_pose_;
    y_0(0) += radio_out->value()[0] * x_scale_;
    y_0(1) += radio_out->value()[1] * y_scale_;
    y_0(2) += radio_out->value()[2] * z_scale_;
    result = drake::trajectories::PiecewisePolynomial<double>(y_0);
    *casted_traj = result;
  } else {
    if (trajectory_input.value(0).isZero()) {
    } else {
      *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
          const PiecewisePolynomial<double>*>(&trajectory_input);
    }
  }
}

}  // namespace dairlib
