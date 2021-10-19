//
// Created by brian on 4/16/21.
//

#include "mpc_trajectory_reciever.h"


namespace dairlib {

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixXd;


MpcTrajectoryReceiver::MpcTrajectoryReceiver(
    TrajectoryType com_type, TrajectoryType swing_ft_type,
    TrajectoryType angular_type, bool planar) :
    com_type_(com_type), angular_type_(angular_type),
    swing_ft_type_(swing_ft_type), planar_(planar),
    kAngularDim_ (planar ? 1 : 3) {

    this->DeclareAbstractInputPort(
        "lcmt_saved_trajectory", drake::Value<lcmt_saved_traj>{});

    PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
    Trajectory<double>& traj_inst = empty_pp_traj;

    com_traj_port_ = this->DeclareAbstractOutputPort("com_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeComTrajFromLcm).get_index();

    angular_traj_port_ = this->DeclareAbstractOutputPort("orientation_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeAngularTrajFromLcm).get_index();

    swing_ft_traj_port_ = this->DeclareAbstractOutputPort("swing_ft_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeSwingFtTrajFromLcm).get_index();
}

void MpcTrajectoryReceiver::MakeAngularTrajFromLcm(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  const auto& input_msg = input->get_value<lcmt_saved_traj>();

  LcmTrajectory lcm_traj(input_msg);
  LcmTrajectory::Trajectory orientation = lcm_traj.GetTrajectory("orientation");

  MatrixXd knots = orientation.datapoints.block(0, 0, kAngularDim_, orientation.datapoints.cols());
  MatrixXd knots_dot = orientation.datapoints.block(kAngularDim_, 0, kAngularDim_, orientation.datapoints.cols());

  auto* casted_traj =
        (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
            traj);
  *casted_traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
        orientation.time_vector, knots);
}

void MpcTrajectoryReceiver::MakeComTrajFromLcm(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  const auto& input_msg = input->get_value<lcmt_saved_traj>();

  LcmTrajectory lcm_traj(input_msg);
  LcmTrajectory::Trajectory com_traj = lcm_traj.GetTrajectory("com_traj");

  MatrixXd knots = planar_ ? Make3dFromPlanar(
      com_traj.datapoints.block(0, 0, 2, com_traj.datapoints.cols())) :
                   com_traj.datapoints.block(0, 0, 3, com_traj.datapoints.cols());

  MatrixXd knots_dot = planar_ ? Make3dFromPlanar(
      com_traj.datapoints.block(2, 0, 2, com_traj.datapoints.cols())) :
                       com_traj.datapoints.block(3, 0, 3, com_traj.datapoints.cols());
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *casted_traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      com_traj.time_vector, knots);
}

void MpcTrajectoryReceiver::MakeSwingFtTrajFromLcm(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  const auto& input_msg = input->get_value<lcmt_saved_traj>();

  LcmTrajectory lcm_traj(input_msg);
  LcmTrajectory::Trajectory swing_ft_traj = lcm_traj.GetTrajectory("swing_foot_traj");

  int cols = swing_ft_traj.datapoints.cols();
  MatrixXd knots = planar_ ? Make3dFromPlanar(
      swing_ft_traj.datapoints.block(0, 0, 2, cols)) :
                   swing_ft_traj.datapoints.block(0, 0, 3, cols);

//  MatrixXd knots_dot = planar_ ? Make3dFromPlanar(
//      swing_ft_traj.datapoints.block(2, 0, 2, cols)) :
//                       swing_ft_traj.datapoints.block(3, 0, 3, cols);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  *casted_traj = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      swing_ft_traj.time_vector, knots);
}

MatrixXd MpcTrajectoryReceiver::Make3dFromPlanar(MatrixXd planar_knots) const {
  DRAKE_ASSERT(planar_knots.rows() == 2);

  MatrixXd knots_3d = MatrixXd::Zero(3, planar_knots.cols());

  knots_3d.block(0, 0, 1, planar_knots.cols()) =
      planar_knots.block(0, 0, 1, planar_knots.cols());
  knots_3d.block(2, 0, 1, planar_knots.cols()) =
      planar_knots.block(1, 0, 1, planar_knots.cols());

  return knots_3d;
}
}