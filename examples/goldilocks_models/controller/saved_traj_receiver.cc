#include "examples/goldilocks_models/controller/saved_traj_receiver.h"

#include <string>

#include "examples/goldilocks_models/controller/control_parameters.h"
#include "lcm/rom_planner_saved_trajectory.h"
#include "multibody/multipose_visualizer.h"
#include "systems/framework/output_vector.h"

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::cout;
using std::endl;

using dairlib::systems::OutputVector;

namespace dairlib {
namespace goldilocks_models {

SavedTrajReceiver::SavedTrajReceiver(
    const ReducedOrderModel& rom,
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    const drake::multibody::MultibodyPlant<double>& plant_control,
    drake::systems::Context<double>* context_feedback,
    const std::vector<BodyPoint>& left_right_foot,
    std::vector<int> left_right_support_fsm_states, bool both_pos_vel_in_traj,
    double single_support_duration, double double_support_duration)
    : ny_(rom.n_y()),
      plant_feedback_(plant_feedback),
      plant_control_(plant_control),
      context_feedback_(context_feedback),
      context_control_(plant_control.CreateDefaultContext()),
      left_right_foot_(left_right_foot),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      nq_(plant_control.num_positions()),
      nv_(plant_control.num_velocities()),
      nx_(plant_control.num_positions() + plant_control.num_velocities()),
      both_pos_vel_in_traj_(both_pos_vel_in_traj),
      single_support_duration_(single_support_duration),
      double_support_duration_(double_support_duration) {
  saved_traj_lcm_port_ =
      this->DeclareAbstractInputPort(
              "saved_traj_lcm",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();
  // Input ports for swing foot
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

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

  // Discrete update for the swing foot at touchdown
  DeclarePerStepDiscreteUpdateEvent(&SavedTrajReceiver::DiscreteVariableUpdate);
  // The swing foot position in the beginning of the swing phase
  liftoff_swing_foot_pos_idx_ = this->DeclareDiscreteState(Vector3d::Zero());

  // Construct maps
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(0)});
}

void SavedTrajReceiver::CalcRomTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj
  auto* traj_casted = dynamic_cast<PiecewisePolynomial<double>*>(traj);

  // Read lcm message
  auto lcm_traj = this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
      context, saved_traj_lcm_port_);

  // Check if traj is empty (if it's empty, it means we are haven't gotten the
  // traj from the planner yet)
  if (lcm_traj->saved_traj.num_trajectories == 0) {
    cout << "WARNING (CalcRomTraj): trajectory size is 0!\n";
    *traj_casted = PiecewisePolynomial<double>(VectorXd::Zero(ny_));
    return;
  }

  // Construct rom planner data from lcm message
  // Benchmark: The unpacking time is about 10-20 us.
  RomPlannerTrajectory traj_data(*lcm_traj);

  // Construct cubic splines
  PiecewisePolynomial<double> pp = traj_data.ConstructPositionTrajectory();

  if (context.get_time() > pp.end_time()) {
    cout << "WARNING: exceeded trajectory's end time! ";
    // cout << "current context time / traj start time / traj end time\n";
    cout << context.get_time() << " / " << pp.start_time() << " / "
         << pp.end_time() << endl;
  }

  // Assign traj
  *traj_casted = pp;
};

EventStatus SavedTrajReceiver::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  VectorXd fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value();

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), int(fsm_state(0)));
  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // when entering a new state which is in left_right_support_fsm_states
  if ((fsm_state(0) != prev_fsm_state_) && is_single_support_phase) {
    prev_fsm_state_ = fsm_state(0);

    auto swing_foot_pos_at_liftoff =
        discrete_state->get_mutable_vector(liftoff_swing_foot_pos_idx_)
            .get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    multibody::SetPositionsIfNew<double>(
        plant_feedback_, robot_output->GetPositions(), context_feedback_);

    // Swing foot position (Forward Kinematics) at touchdown
    const BodyPoint& swing_foot = swing_foot_map_.at(int(fsm_state(0)));
    plant_feedback_.CalcPointsPositions(
        *context_feedback_, swing_foot.second, swing_foot.first,
        plant_feedback_.world_frame(), &swing_foot_pos_at_liftoff);

    lift_off_time_ = robot_output->get_timestamp();
  }

  return EventStatus::Succeeded();
}

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

  // Cast traj
  auto* traj_casted = dynamic_cast<PiecewisePolynomial<double>*>(traj);

  // Read the lcm message
  auto lcm_traj = this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
      context, saved_traj_lcm_port_);

  // Check if traj is empty (if it's empty, it means we are haven't gotten the
  // traj from the planner yet)
  if (lcm_traj->saved_traj.num_trajectories == 0) {
    cout << "WARNING (CalcSwingFootTraj): trajectory size is 0!\n";
    *traj_casted = PiecewisePolynomial<double>(Vector3d::Zero());
    return;
  }

  // Construct rom planner data from lcm message
  RomPlannerTrajectory traj_data(*lcm_traj);
  int n_mode = traj_data.GetNumModes();

  // Get states (in global frame) and stance_foot
  const MatrixXd& x0 = traj_data.get_x0();
  const VectorXd& x0_time = traj_data.get_x0_time();
  const MatrixXd& xf = traj_data.get_xf();
  const VectorXd& xf_time = traj_data.get_xf_time();
  const VectorXd& stance_foot = traj_data.get_stance_foot();
  DRAKE_DEMAND(xf_time(0) == x0_time(1));

  // Construct PP
  PiecewisePolynomial<double> pp;
  std::vector<double> T_waypoint = std::vector<double>(3, 0);
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  Vector3d foot_pos;
  bool left_stance = abs(stance_foot(0)) < 1e-12;
  for (int j = 0; j < n_mode; j++) {
    if (xf_time(j) - double_support_duration_ > x0_time(j)) {
      // T_waypoint.at(0) = (j == 0) ? lift_off_time_ : x0_time(j);
      T_waypoint.at(0) = (j == 0) ? xf_time(j) - single_support_duration_ -
                                        double_support_duration_
                                  : x0_time(j);
      T_waypoint.at(2) = xf_time(j) - double_support_duration_;
      /*cout << "T_waypoint.at(0) = " << T_waypoint.at(0) << endl;
      cout << "T_waypoint.at(2) = " << T_waypoint.at(2) << endl;
      cout << "double_support_duration_ = " << double_support_duration_ <<
      endl;*/
      T_waypoint.at(1) = (T_waypoint.at(0) + T_waypoint.at(2)) / 2;
      plant_control_.SetPositionsAndVelocities(context_control_.get(),
                                               x0.col(j));
      if (j == 0) {
        foot_pos =
            context.get_discrete_state(liftoff_swing_foot_pos_idx_).get_value();
      } else {
        plant_control_.CalcPointsPositions(
            *context_control_, left_right_foot_.at(left_stance ? 1 : 0).second,
            left_right_foot_.at(left_stance ? 1 : 0).first,
            plant_control_.world_frame(), &foot_pos);
      }
      Y.at(0) = foot_pos;
      plant_control_.SetPositionsAndVelocities(context_control_.get(),
                                               xf.col(j));
      plant_control_.CalcPointsPositions(
          *context_control_, left_right_foot_.at(left_stance ? 1 : 0).second,
          left_right_foot_.at(left_stance ? 1 : 0).first,
          plant_control_.world_frame(), &foot_pos);
      Y.at(2) = foot_pos;
      Y.at(1) = (Y.at(0) + Y.at(2)) / 2;
      Y.at(1)(2) += 0.1;
      // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to
      // make the traj smooth at the mid point
      pp.ConcatenateInTime(
          PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
              T_waypoint, Y, VectorXd::Zero(3), VectorXd::Zero(3)));
    }

    // Fill in the double support phase with a constant zero traj
    if (double_support_duration_ > 0) {
      VectorXd T_double_support(2);
      T_double_support << T_waypoint.at(2), xf_time(j);
      /*cout << "T_waypoint.at(2) = " << T_waypoint.at(2) << endl;
      cout << "xf_time(j) = " << xf_time(j) << endl;*/
      MatrixXd Y_double_support = MatrixXd::Zero(3, 2);
      pp.ConcatenateInTime(PiecewisePolynomial<double>::ZeroOrderHold(
          T_double_support, Y_double_support));
    }

    left_stance = !left_stance;
  }

  // Assign traj
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
