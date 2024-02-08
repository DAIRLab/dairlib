#include "examples/Cassie/osc/standing_pelvis_orientation_traj.h"

#include <dairlib/lcmt_cassie_out.hpp>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/wrap_to.h"

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace dairlib::cassie::osc {

StandingPelvisOrientationTraj::StandingPelvisOrientationTraj(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::vector<std::pair<const Eigen::Vector3d,
                                const drake::multibody::Frame<double>&>>&
        feet_contact_points,
    const std::string& traj_name)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      feet_contact_points_(feet_contact_points) {
  target_orientation_filter_ =
      std::make_unique<FirstOrderLowPassFilter>(1.0, 3);

  // Input/Output setup
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();
  radio_port_ = this->DeclareAbstractInputPort(
                        "radio_out", drake::Value<dairlib::lcmt_radio_out>{})
                    .get_index();
  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->set_name(traj_name);
  this->DeclareAbstractOutputPort(traj_name, traj_inst,
                                  &StandingPelvisOrientationTraj::CalcTraj);
}

void StandingPelvisOrientationTraj::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  VectorXd q = robot_output->GetPositions();
  plant_.SetPositions(context_, q);
  auto* casted_traj =
      (PiecewiseQuaternionSlerp<double>*)dynamic_cast<PiecewiseQuaternionSlerp<double>*>(
          traj);
  Vector3d pt_0;
  Vector3d pt_1;
  Vector3d pt_2;
  Vector3d pt_3;
  plant_.CalcPointsPositions(*context_, feet_contact_points_[0].second,
                             feet_contact_points_[0].first, world_, &pt_0);
  plant_.CalcPointsPositions(*context_, feet_contact_points_[1].second,
                             feet_contact_points_[1].first, world_, &pt_1);
  plant_.CalcPointsPositions(*context_, feet_contact_points_[2].second,
                             feet_contact_points_[2].first, world_, &pt_2);
  plant_.CalcPointsPositions(*context_, feet_contact_points_[3].second,
                             feet_contact_points_[3].first, world_, &pt_3);
  Vector3d ground_plane;

  VectorXd l_foot = pt_0 - pt_1;
  VectorXd r_foot = pt_2 - pt_3;
  //  l_foot_proj = l_foot.dot()
  Vector3d rpy;
  rpy << radio_out->channel[1], radio_out->channel[2],
      drake::math::wrap_to(
          0.5 * (atan2(l_foot(1), l_foot(0)) + atan2(r_foot(1), r_foot(0))),
          -M_PI/4, M_PI/4) +
          radio_out->channel[3];

  target_orientation_filter_->Update(rpy);
  auto rot_mat = drake::math::RotationMatrix<double>(drake::math::RollPitchYaw(
      static_cast<Vector3d>(target_orientation_filter_->Value())));
  const std::vector<double> breaks = {0, 1};
  *casted_traj = drake::trajectories::PiecewiseQuaternionSlerp<double>(
      breaks, {rot_mat.ToQuaternion(), rot_mat.ToQuaternion()});
}

}  // namespace dairlib::cassie::osc
