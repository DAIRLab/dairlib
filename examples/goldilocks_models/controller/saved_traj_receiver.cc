#include "examples/goldilocks_models/controller/saved_traj_receiver.h"

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;

namespace dairlib {
namespace goldilocks_models {

SavedTrajReceiver::SavedTrajReceiver(int n_tail_ignored, bool use_exp,
                                     bool both_pos_vel_in_traj)
    : n_tail_ignored_(n_tail_ignored),
      use_exp_(use_exp),
      both_pos_vel_in_traj_(both_pos_vel_in_traj) {
  saved_traj_lcm_port_ =
      this->DeclareAbstractInputPort("saved_traj_lcm",
                                     drake::Value<dairlib::lcmt_saved_traj>{})
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  if (use_exp) {
    ExponentialPlusPiecewisePolynomial<double> exp;
    drake::trajectories::Trajectory<double>& traj_inst = exp;
    this->DeclareAbstractOutputPort("saved_traj", traj_inst,
                                    &SavedTrajReceiver::CalcDesiredTraj);

  } else {
    PiecewisePolynomial<double> pp;
    drake::trajectories::Trajectory<double>& traj_inst = pp;
    this->DeclareAbstractOutputPort("saved_traj", traj_inst,
                                    &SavedTrajReceiver::CalcDesiredTraj);
  }
};

void SavedTrajReceiver::CalcDesiredTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Construct traj from lcm message
  LcmTrajectory traj_data(*(this->EvalInputValue<dairlib::lcmt_saved_traj>(
      context, saved_traj_lcm_port_)));
  const auto& traj_names = traj_data.GetTrajectoryNames();

  int n_mode = traj_names.size() - n_tail_ignored_;
  int n_y = traj_data.GetTrajectory(traj_names[0]).datatypes.size();
  if (both_pos_vel_in_traj_) n_y /= 2;

  VectorXd zero_vec = VectorXd::Zero(n_y);

  // TODO: looks like you can simplify the code below. Just create an empty traj
  //  frist and than concatenate. Ref:
  //  https://github.com/RobotLocomotion/drake/blob/b09e40db4b1c01232b22f7705fb98aa99ef91f87/common/trajectories/piecewise_polynomial.cc#L303

  const LcmTrajectory::Trajectory& traj0 =
      traj_data.GetTrajectory(traj_names[0]);
  PiecewisePolynomial<double> pp_part =
      both_pos_vel_in_traj_
          ? PiecewisePolynomial<double>::CubicHermite(
                traj0.time_vector, traj0.datapoints.topRows(n_y),
                traj0.datapoints.bottomRows(n_y))
          : PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
                traj0.time_vector, traj0.datapoints, zero_vec, zero_vec);
  for (int mode = 1; mode < n_mode; ++mode) {
    const LcmTrajectory::Trajectory& traj_i =
        traj_data.GetTrajectory(traj_names[mode]);
    pp_part.ConcatenateInTime(
        both_pos_vel_in_traj_
            ? PiecewisePolynomial<double>::CubicHermite(
                  traj_i.time_vector, traj_i.datapoints.topRows(n_y),
                  traj_i.datapoints.bottomRows(n_y))
            : PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
                  traj_i.time_vector, traj_i.datapoints, zero_vec, zero_vec));
  }

  // Cast traj and assign traj
  if (use_exp_) {
    auto* traj_casted =
        (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
            ExponentialPlusPiecewisePolynomial<double>*>(traj);
    *traj_casted = ExponentialPlusPiecewisePolynomial<double>(pp_part);
  } else {
    auto* traj_casted = (PiecewisePolynomial<double>*)dynamic_cast<
        PiecewisePolynomial<double>*>(traj);
    *traj_casted = pp_part;
  }
};

IKTrajReceiver::IKTrajReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<std::string>& ordered_pos_names)
    : plant_(plant), nq_(plant.num_positions()) {
  saved_traj_lcm_port_ =
      this->DeclareAbstractInputPort("saved_traj_lcm",
                                     drake::Value<dairlib::lcmt_saved_traj>{})
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp;
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("saved_traj", traj_inst,
                                  &IKTrajReceiver::CalcDesiredTraj);

  // Construct the index list
  auto pos_idx_map = multibody::makeNameToPositionsMap(plant);
  ordered_indices_.clear();
  for (const auto& pos_name : ordered_pos_names) {
    ordered_indices_.push_back(pos_idx_map.at(pos_name));
  }
};

void IKTrajReceiver::CalcDesiredTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Construct traj from lcm message
  LcmTrajectory traj_data(*(this->EvalInputValue<dairlib::lcmt_saved_traj>(
      context, saved_traj_lcm_port_)));
  const auto& traj_names = traj_data.GetTrajectoryNames();

  int n_mode = traj_names.size();

  // The code assumes that the output of IK sends the full configuration of the
  // robot. (currently the one without springs)
  DRAKE_DEMAND(nq_ == traj_data.GetTrajectory(traj_names[0]).datatypes.size());

  int n_y = ordered_indices_.size();

  VectorXd zero_vec = VectorXd::Zero(n_y);

  // TODO: looks like you can simplify the code below. Just create an empty traj
  //  frist and than concatenate. Ref:
  //  https://github.com/RobotLocomotion/drake/blob/b09e40db4b1c01232b22f7705fb98aa99ef91f87/common/trajectories/piecewise_polynomial.cc#L303

  const LcmTrajectory::Trajectory& traj0 =
      traj_data.GetTrajectory(traj_names[0]);
  Eigen::MatrixXd extracted_data0(n_y, traj0.time_vector.size());
  for (int i = 0; i < ordered_indices_.size(); i++) {
    extracted_data0.row(i) = traj0.datapoints.row(ordered_indices_.at(i));
  }
  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          traj0.time_vector, extracted_data0, zero_vec, zero_vec);
  for (int mode = 1; mode < n_mode; ++mode) {
    const LcmTrajectory::Trajectory& traj_i =
        traj_data.GetTrajectory(traj_names[mode]);
    Eigen::MatrixXd extracted_data_i(n_y, traj_i.time_vector.size());
    for (int i = 0; i < ordered_indices_.size(); i++) {
      extracted_data_i.row(i) = traj_i.datapoints.row(ordered_indices_.at(i));
    }
    pp_part.ConcatenateInTime(
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            traj_i.time_vector, extracted_data_i, zero_vec, zero_vec));
  }

  // Cast traj and assign traj
  auto* traj_casted =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *traj_casted = pp_part;
};

}  // namespace goldilocks_models
}  // namespace dairlib
