#include "lcm_trajectory_systems.h"

#include <iostream>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/geometry/rgba.h"

namespace dairlib {
namespace systems {

using drake::geometry::Rgba;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

LcmTrajectoryReceiver::LcmTrajectoryReceiver(std::string trajectory_name)
    : trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->set_name(trajectory_name_);
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort(trajectory_name_, traj_inst,
                                      &LcmTrajectoryReceiver::OutputTrajectory)
          .get_index();
  lcm_traj_ = LcmTrajectory(dairlib::FindResourceOrThrow(nominal_stand_path_));
}

void LcmTrajectoryReceiver::OutputTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcm_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    lcm_traj_ = LcmTrajectory(lcm_traj->saved_traj);
  }
  const auto trajectory_block = lcm_traj_.GetTrajectory(trajectory_name_);
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
      trajectory_block.time_vector, trajectory_block.datapoints);
}

LcmTrajectoryDrawer::LcmTrajectoryDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    std::string trajectory_name)
    : meshcat_(meshcat), trajectory_name_(std::move(trajectory_name)) {
  trajectory_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  lcm_traj_ = LcmTrajectory(dairlib::FindResourceOrThrow(nominal_stand_path_));
  DeclarePerStepDiscreteUpdateEvent(&LcmTrajectoryDrawer::DrawTrajectory);
}

drake::systems::EventStatus LcmTrajectoryDrawer::DrawTrajectory(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, trajectory_input_port_)
          ->utime > 1e-3) {
    const auto& lcm_traj =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, trajectory_input_port_);
    lcm_traj_ = LcmTrajectory(lcm_traj->saved_traj);
  }
  Eigen::MatrixXd setpoints = lcm_traj_.GetTrajectory(trajectory_name_).datapoints;
  for(int i = 0; i < setpoints.cols(); ++i){
    setpoints(2, i) += 0.7645;
  }
  meshcat_->SetLine("/trajectories/end_effector_target",
                    setpoints, 100000,
                    Rgba(0.1, 0.1, 0.1, 1.0));
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace dairlib
