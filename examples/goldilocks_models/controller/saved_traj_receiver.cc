#include "examples/goldilocks_models/controller/saved_traj_receiver.h"

#include "examples/goldilocks_models/controller/control_parameters.h"
#include "lcm/rom_planner_saved_trajectory.h"

#include <string>

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::cout;
using std::endl;

namespace dairlib {
namespace goldilocks_models {

SavedTrajReceiver::SavedTrajReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<BodyPoint>& left_right_foot, bool both_pos_vel_in_traj,
    double double_support_duration)
    : plant_control_(plant),
      left_right_foot_(left_right_foot),
      context_(plant.CreateDefaultContext()),
      nq_(plant.num_positions()),
      nv_(plant.num_velocities()),
      nx_(plant.num_positions() + plant.num_velocities()),
      both_pos_vel_in_traj_(both_pos_vel_in_traj),
      double_support_duration_(double_support_duration) {
  saved_traj_lcm_port_ =
      this->DeclareAbstractInputPort("saved_traj_lcm",
                                     drake::Value<dairlib::lcmt_saved_traj>{})
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp;
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  rom_traj_port_ =
      this->DeclareAbstractOutputPort("rom_traj", traj_inst,
                                      &SavedTrajReceiver::CalcRomTraj)
          .get_index();
  PiecewisePolynomial<double> pp2;
  drake::trajectories::Trajectory<double>& traj_inst2 = pp2;
  swing_foot_traj_port_ =
      this->DeclareAbstractOutputPort("swing_foot_traj", traj_inst2,
                                      &SavedTrajReceiver::CalcSwingFootTraj)
          .get_index();
}

void SavedTrajReceiver::CalcRomTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Construct rom planner data from lcm message
  // Benchmark: The unpacking time is about 10-20 us.
  RomPlannerTrajectory traj_data(
      *(this->EvalInputValue<dairlib::lcmt_saved_traj>(context,
                                                       saved_traj_lcm_port_)));

  int n_mode = traj_data.GetNumModes();
  int n_y = traj_data.GetTrajectory("state_traj0").datatypes.size();
  if (both_pos_vel_in_traj_) n_y /= 2;

  VectorXd zero_vec = VectorXd::Zero(n_y);

  PiecewisePolynomial<double> pp_part;
  for (int mode = 0; mode < n_mode; ++mode) {
    const LcmTrajectory::Trajectory& traj_i =
        traj_data.GetTrajectory("state_traj" + std::to_string(mode));
    pp_part.ConcatenateInTime(
        both_pos_vel_in_traj_
            ? PiecewisePolynomial<double>::CubicHermite(
                  traj_i.time_vector, traj_i.datapoints.topRows(n_y),
                  traj_i.datapoints.bottomRows(n_y))
            : PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
                  traj_i.time_vector, traj_i.datapoints, zero_vec, zero_vec));
  }

  // Cast traj and assign traj
  auto* traj_casted = dynamic_cast<PiecewisePolynomial<double>*>(traj);
  *traj_casted = pp_part;
};

void SavedTrajReceiver::CalcSwingFootTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // TODO: You can save time by moving CalcRomTraj and CalcSwingFootTraj to
  //  a perstep update, and only update the trajs (internal states) when the
  //  start time of the new traj is different. And the output functions of this
  //  leafsystem only copies the state

  // TODO: currently we set the touchdown time to be at the end of single
  //  support, but this is not consistent with the touchdown time in the
  //  planner. Should find a way to incorporate the double support phase.

  // TODO: the code in CalcSwingFootTraj hasn't been tested yet

  // We assume the start and the end velocity of the swing foot are 0

  // Construct rom planner data from lcm message
  RomPlannerTrajectory traj_data(
      *(this->EvalInputValue<dairlib::lcmt_saved_traj>(context,
                                                       saved_traj_lcm_port_)));
  int n_mode = traj_data.GetNumModes();

  // Get states and stance_foot
  // Not sure if we are actually using reference here
  const MatrixXd& x0 = traj_data.get_x0_FOM()->datapoints;
  const VectorXd& x0_time = traj_data.get_x0_FOM()->time_vector;
  const MatrixXd& xf = traj_data.get_xf_FOM()->datapoints;
  const VectorXd& xf_time = traj_data.get_xf_FOM()->time_vector;
  const VectorXd& stance_foot = traj_data.get_stance_foot();
  const VectorXd& quat_xyz_shift = traj_data.get_quat_xyz_shift();
  DRAKE_DEMAND(xf_time(0) == x0_time(1));

  // Rotate the states back to global frame
  MatrixXd x0_global = x0;
  MatrixXd xf_global = xf;
  Quaterniond relative_quat = Quaterniond(quat_xyz_shift(0), quat_xyz_shift(1),
                                          quat_xyz_shift(2), quat_xyz_shift(3))
                                  .conjugate();
  Matrix3d relative_rot_mat = relative_quat.toRotationMatrix();
  for (int j = 0; j < n_mode; j++) {
    // x0
    Quaterniond x0_quat_global =
        relative_quat * Quaterniond(x0_global.col(j)(0), x0_global.col(j)(1),
                                    x0_global.col(j)(2), x0_global.col(j)(3));
    x0_global.col(j).segment<4>(0) << x0_quat_global.w(), x0_quat_global.vec();
    x0_global.col(j).segment<3>(4)
        << x0_global.col(j).segment<3>(4) - quat_xyz_shift.segment<3>(4);
    x0_global.col(j).segment<3>(nq_)
        << relative_rot_mat * x0_global.col(j).segment<3>(nq_);
    x0_global.col(j).segment<3>(nq_ + 3)
        << relative_rot_mat * x0_global.col(j).segment<3>(nq_ + 3);
    // xf
    Quaterniond xf_quat_global =
        relative_quat * Quaterniond(xf_global.col(j)(0), xf_global.col(j)(1),
                                    xf_global.col(j)(2), xf_global.col(j)(3));
    xf_global.col(j).segment<4>(0) << xf_quat_global.w(), xf_quat_global.vec();
    xf_global.col(j).segment<3>(4)
        << xf_global.col(j).segment<3>(4) - quat_xyz_shift.segment<3>(4);
    xf_global.col(j).segment<3>(nq_)
        << relative_rot_mat * xf_global.col(j).segment<3>(nq_);
    xf_global.col(j).segment<3>(nq_ + 3)
        << relative_rot_mat * xf_global.col(j).segment<3>(nq_ + 3);
  }

  // Construct PP
  PiecewisePolynomial<double> pp;
  std::vector<double> T_waypoint = std::vector<double>(3, 0);
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  Vector3d foot_pos;
  bool left_stance = abs(stance_foot(0)) < 1e-12;
  for (int j = 0; j < n_mode; j++) {
    if (xf_time(j) - double_support_duration_ > x0_time(j)) {
      T_waypoint.at(0) = x0_time(j);
      T_waypoint.at(2) = xf_time(j) - double_support_duration_;
      /*cout << "T_waypoint.at(0) = " << T_waypoint.at(0) << endl;
      cout << "T_waypoint.at(2) = " << T_waypoint.at(2) << endl;
      cout << "double_support_duration_ = " << double_support_duration_ <<
      endl;*/
      T_waypoint.at(1) = (T_waypoint.at(0) + T_waypoint.at(2)) / 2;
      plant_control_.SetPositionsAndVelocities(context_.get(),
                                               x0_global.col(j));
      plant_control_.CalcPointsPositions(
          *context_, left_right_foot_.at(left_stance ? 1 : 0).second,
          left_right_foot_.at(left_stance ? 1 : 0).first,
          plant_control_.world_frame(), &foot_pos);
      Y.at(0) = foot_pos;
      plant_control_.SetPositionsAndVelocities(context_.get(),
                                               xf_global.col(j));
      plant_control_.CalcPointsPositions(
          *context_, left_right_foot_.at(left_stance ? 1 : 0).second,
          left_right_foot_.at(left_stance ? 1 : 0).first,
          plant_control_.world_frame(), &foot_pos);
      Y.at(2) = foot_pos;
      Y.at(1) = (Y.at(0) + Y.at(2)) / 2;
      Y.at(1)(1) += 0.1;
      // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to
      // make the traj smooth at the mid point
      pp.ConcatenateInTime(
          PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
              T_waypoint, Y, VectorXd::Zero(3), VectorXd::Zero(3)));
    }

    // Fill in the double support phase with a constant zero traj
    VectorXd T_double_support(2);
    T_double_support << T_waypoint.at(2), xf_time(j);
    /*cout << "T_waypoint.at(2) = " << T_waypoint.at(2) << endl;
    cout << "xf_time(j) = " << xf_time(j) << endl;*/
    MatrixXd Y_double_support = MatrixXd::Zero(3, 2);
    pp.ConcatenateInTime(PiecewisePolynomial<double>::ZeroOrderHold(
        T_double_support, Y_double_support));

    left_stance = !left_stance;
  }

  // Cast traj and assign traj
  auto* traj_casted = dynamic_cast<PiecewisePolynomial<double>*>(traj);
  *traj_casted = pp;
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
