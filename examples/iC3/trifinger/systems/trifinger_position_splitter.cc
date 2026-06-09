#include "trifinger_position_splitter.h"

#include <iostream>


#include "common/eigen_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib {

using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

TrifingerPositionSplitter::TrifingerPositionSplitter(std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->set_name(trajectory_name_);

  finger_0_output_port_ =
    this->DeclareAbstractOutputPort(
        "finger_0_output",
        traj_inst,
        &TrifingerPositionSplitter::OutputTrajectory0)
      .get_index();

  finger_120_output_port_ =
    this->DeclareAbstractOutputPort(
        "finger_120_output",
        traj_inst,
        &TrifingerPositionSplitter::OutputTrajectory120)
      .get_index();

  finger_240_output_port_ =
    this->DeclareAbstractOutputPort(
        "finger_240_output",
        traj_inst,
        &TrifingerPositionSplitter::OutputTrajectory240)
      .get_index(); 
}

void TrifingerPositionSplitter::OutputTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj, int finger_idx) const {
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-5) {
    const auto& lcmt_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    auto lcm_traj = LcmTrajectory(lcmt_traj->saved_traj);
    const auto& trajectory_block = lcm_traj.GetTrajectory(trajectory_name_);
      
    MatrixXd data = trajectory_block.datapoints.middleRows(3*finger_idx, 3);
    std::cout << data << std::endl;

    *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
        trajectory_block.time_vector, data);

  } else {
    *casted_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}

void TrifingerPositionSplitter::OutputTrajectory0(
  const drake::systems::Context<double>& context,
  drake::trajectories::Trajectory<double>* traj) const {
  OutputTrajectory(context, traj, 0);
}

void TrifingerPositionSplitter::OutputTrajectory120(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  OutputTrajectory(context, traj, 1);
}

void TrifingerPositionSplitter::OutputTrajectory240(
  const drake::systems::Context<double>& context,
  drake::trajectories::Trajectory<double>* traj) const {
  OutputTrajectory(context, traj, 2);
}                          

}  // namespace dairlib
