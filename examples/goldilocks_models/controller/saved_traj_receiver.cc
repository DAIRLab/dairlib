#include "examples/goldilocks_models/controller/saved_traj_receiver.h"

#include "examples/goldilocks_models/controller/control_parameters.h"
#include "lcm/rom_planner_saved_trajectory.h"

#include <string>

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;

namespace dairlib {
namespace goldilocks_models {

SavedTrajReceiver::SavedTrajReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    bool both_pos_vel_in_traj)
    : plant_control_(plant),
      nq_(plant.num_positions()),
      nv_(plant.num_velocities()),
      nx_(plant.num_positions() + plant.num_velocities()),
      both_pos_vel_in_traj_(both_pos_vel_in_traj) {
  saved_traj_lcm_port_ =
      this->DeclareAbstractInputPort("saved_traj_lcm",
                                     drake::Value<dairlib::lcmt_saved_traj>{})
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp;
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("saved_traj", traj_inst,
                                  &SavedTrajReceiver::CalcDesiredTraj);
}

void SavedTrajReceiver::CalcDesiredTraj(
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
  // TODO: You can save time by moving CalcDesiredTraj and CalcSwingFootTraj to
  //  a perstep update, and only update the trajs (internal states) when the
  //  start time of the new traj is different. And the output functions of this
  //  leafsystem only copies the state

  // TODO: currently we set the touchdown time to be at the end of single
  //  support, but this is not consistent with the touchdown time in the
  //  planner. Should find a way to incorporate the double support phase.

  // TODO: note that we haven't rotated the velocity back to the global frame

  // Construct rom planner data from lcm message
  RomPlannerTrajectory traj_data(
      *(this->EvalInputValue<dairlib::lcmt_saved_traj>(context,
                                                       saved_traj_lcm_port_)));

  // Get states and stance_foot
  const MatrixXd& x0 = traj_data.get_x0_FOM()->datapoints;
  const MatrixXd& xf = traj_data.get_xf_FOM()->datapoints;
  const VectorXd& stance_foot = traj_data.get_stance_foot();
  const VectorXd& quat_xyz_shift = traj_data.get_quat_xyz_shift();
  DRAKE_DEMAND(traj_data.get_x0_FOM()->time_vector(1) ==
               traj_data.get_x0_FOM()->time_vector(2));

  // TODO: Rotate the states back to global frame

  // Construct PP
  PiecewisePolynomial<double> pp;

  int n_mode = traj_data.GetNumModes();
  bool left_stance = abs(stance_foot(0)) < 1e-12;
  for (int j = 0; j < n_mode; j++) {
    // We assume the start and the end vel of the swing foot are 0

    // TODO: construct swing foot traj

    // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to
    // make the traj smooth at the mid point
    /*PiecewisePolynomial<double> swing_foot_spline =
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            T_waypoint, Y, Y_dot.at(0), Y_dot.at(2));*/

    left_stance = !left_stance;
  }

  // Cast traj and assign traj
  auto* traj_casted =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
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
