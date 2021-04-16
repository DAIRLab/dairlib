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
    swing_ft_type_(swing_ft_type), planar_(planar) {

    this->DeclareAbstractInputPort(
        "lcmt_saved_trajectory", drake::Value<lcmt_saved_traj>{});

    PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
    Trajectory<double>& traj_inst = empty_pp_traj;

    com_traj_port_ = this->DeclareAbstractOutputPort("com_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeComTrajFromLcm).get_index();

    angular_traj_port_ = this->DeclareAbstractOutputPort("angular_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeAngularTrajFromLcm).get_index();

    swing_ft_traj_port_ = this->DeclareAbstractOutputPort("swing_ft_traj",
        traj_inst, &MpcTrajectoryReceiver::MakeSwingFtTrajFromLcm).get_index();
}

void MpcTrajectoryReceiver::MakeComTrajFromLcm(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

}

void MpcTrajectoryReceiver::MakeAngularTrajFromLcm(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

}

void MpcTrajectoryReceiver::MakeSwingFtTrajFromLcm(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *traj) const {

}

}