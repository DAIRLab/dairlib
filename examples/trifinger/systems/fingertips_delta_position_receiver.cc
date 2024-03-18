#include "fingertips_delta_position_receiver.h"

#include <iostream>
#include <utility>

namespace dairlib::systems {
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

FingertipDeltaPositionReceiver::FingertipDeltaPositionReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const Eigen::Vector3d& min_fingertips_delta_position,
    const Eigen::Vector3d& max_fingertips_delta_position,
    const std::string& fingertip_0_name, const std::string& fingertip_120_name,
    const std::string& fingertip_240_name)
    : plant_(plant),
      context_(context),
      min_fingertips_delta_position_(min_fingertips_delta_position),
      max_fingertips_delta_position_(max_fingertips_delta_position),
      fingertip_0_name_(fingertip_0_name),
      fingertip_120_name_(fingertip_120_name),
      fingertip_240_name_(fingertip_240_name) {
  this->set_name("fingertips_delta_position_receiver");
  state_port_ =
      this->DeclareVectorInputPort("x_trifinger",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();

  fingertips_delta_position_port_ =
      this->DeclareAbstractInputPort(
              "fingertips_delta_position",
              drake::Value<dairlib::lcmt_fingertips_delta_position>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::Vector3d::Zero());
  Trajectory<double>& traj_inst = empty_pp_traj;
  fingertips_target_port_ =
      this->DeclareAbstractOutputPort(
              "fingertips_target_traj", traj_inst,
              &FingertipDeltaPositionReceiver::CalcFingertipsPositionTargetTraj)
          .get_index();
}

void FingertipDeltaPositionReceiver::CalcFingertipsPositionTargetTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* target_traj) const {
  const OutputVector<double>* trifinger_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const auto& fingertips_delta_pos_lcm_msg =
      this->EvalInputValue<dairlib::lcmt_fingertips_delta_position>(
          context, fingertips_delta_position_port_);
  Eigen::VectorXd q_trifinger = trifinger_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q_trifinger, context_);

  auto fingertip_0_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_0_name_))
          .translation();
  auto fingertip_120_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_120_name_))
          .translation();
  auto fingertip_240_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_240_name_))
          .translation();
  Eigen::VectorXd fingertips_target_pos(9);
  fingertips_target_pos << fingertip_0_pos, fingertip_120_pos,
      fingertip_240_pos;
  fingertips_target_pos +=
      Eigen::VectorXd::Map(fingertips_delta_pos_lcm_msg->deltaPos, 9);
  PiecewisePolynomial<double> const_traj(fingertips_target_pos);

  auto casted_target_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          target_traj);
  *casted_target_traj = const_traj;
}

}  // namespace dairlib::systems