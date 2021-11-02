#include "mpc_trajectory_reciever.h"


namespace dairlib {

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using drake::systems::BasicVector;
using drake::systems::Context;

using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::MatrixXd;


MpcTrajectoryReceiver::MpcTrajectoryReceiver(
    TrajectoryType com_type,
    TrajectoryType angular_type, bool planar) :
    com_type_(com_type),
    angular_type_(angular_type),
    planar_(planar),
    kAngularDim_ (planar ? 1 : 3) {

    this->DeclareAbstractInputPort(
        "lcmt_saved_trajectory", drake::Value<lcmt_saved_traj>{});

    PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
    Trajectory<double>& traj_inst = empty_pp_traj;

    com_traj_port_ = this->DeclareAbstractOutputPort("com_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeComTrajFromLcm).get_index();

    angular_traj_port_ = this->DeclareAbstractOutputPort("orientation_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeAngularTrajFromLcm).get_index();

    swing_ft_traj_port_ = this->DeclareVectorOutputPort(
        "swing_ft_traj", BasicVector<double>(Vector3d::Zero()),
            &MpcTrajectoryReceiver::MakeSwingFtTargetFromLcm).get_index();
}

void MpcTrajectoryReceiver::MakeAngularTrajFromLcm(
    const Context<double> &context, Trajectory<double> *traj) const {

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
  *casted_traj = PiecewisePolynomial<double>::CubicHermite(
        orientation.time_vector, knots, knots_dot);
}

void MpcTrajectoryReceiver::MakeComTrajFromLcm(
    const Context<double> &context, Trajectory<double> *traj) const {

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
  *casted_traj = PiecewisePolynomial<double>::CubicHermite(
      com_traj.time_vector, knots, knots_dot);
}

void MpcTrajectoryReceiver::MakeSwingFtTargetFromLcm(
    const Context<double>& context, BasicVector<double>* target) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  const auto& input_msg = input->get_value<lcmt_saved_traj>();

  LcmTrajectory lcm_traj(input_msg);
  LcmTrajectory::Trajectory swing_ft_traj = lcm_traj.GetTrajectory("swing_foot_traj");

  target->set_value(swing_ft_traj.datapoints.block(0,0,3,1));
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