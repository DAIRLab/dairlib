#include "examples/goldilocks_models/controller/saved_traj_receiver.h"

#include <string>

#include "common/file_utils.h"
#include "examples/goldilocks_models/controller/control_parameters.h"
#include "lcm/hybrid_rom_planner_saved_trajectory.h"
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
    std::vector<int> left_right_support_fsm_states,
    double single_support_duration, double double_support_duration,
    double desired_mid_foot_height, double desired_final_foot_height,
    const RomWalkingGains& gains,
    const StateMirror& state_mirror /*Only use for sim gap testing*/,
    const multibody::WorldYawViewFrame<double>& view_frame_feedback,
    const multibody::WorldYawViewFrame<double>& view_frame_control,
    bool wrt_com_in_local_frame, bool use_hybrid_rom_mpc)
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
      single_support_duration_(single_support_duration),
      double_support_duration_(double_support_duration),
      desired_mid_foot_height_(desired_mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      view_frame_feedback_(view_frame_feedback),
      view_frame_control_(view_frame_control),
      wrt_com_in_local_frame_(wrt_com_in_local_frame),
      use_hybrid_rom_mpc_(use_hybrid_rom_mpc) {
  if (use_hybrid_rom_mpc_ && wrt_com_in_local_frame_) {
    cout << "Warning: swing foot position relative to the COM is expressed in "
            "the current pelvis frame instead of the touchdown event's \n";
  }

  saved_traj_lcm_port_ =
      this->DeclareAbstractInputPort(
              "saved_traj_lcm",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();
  // Input ports for swing foot
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant_feedback.num_positions(),
                                              plant_feedback.num_velocities(),
                                              plant_feedback.num_actuators()))
          .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();

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
  PiecewisePolynomial<double> pp3;
  drake::trajectories::Trajectory<double>& traj_inst3 = pp3;
  hip_rpy_traj_port_ =
      this->DeclareAbstractOutputPort("stance_hip_rpy_traj", traj_inst3,
                                      &SavedTrajReceiver::CalcStanceHipTraj)
          .get_index();
  PiecewisePolynomial<double> pp4;
  drake::trajectories::Trajectory<double>& traj_inst4 = pp4;
  swing_hip_yaw_traj_port_ =
      this->DeclareAbstractOutputPort("swing_hip_yaw_traj", traj_inst4,
                                      &SavedTrajReceiver::CalcSwingHipTraj)
          .get_index();

  // Discrete update for the swing foot at touchdown
  DeclarePerStepDiscreteUpdateEvent(&SavedTrajReceiver::DiscreteVariableUpdate);
  // The swing foot position in the beginning of the swing phase
  liftoff_swing_foot_pos_idx_ = this->DeclareDiscreteState(Vector3d::Zero());
  // pos and vel index
  liftoff_stance_hip_pos_idx_ = this->DeclareDiscreteState(Vector3d::Zero());
  liftoff_stance_hip_vel_idx_ = this->DeclareDiscreteState(Vector3d::Zero());
  liftoff_swing_hip_pos_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));
  liftoff_swing_hip_vel_idx_ = this->DeclareDiscreteState(VectorXd::Zero(1));

  // Construct body maps
  swing_foot_map1_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(1)});
  swing_foot_map1_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(0)});

  std::map<std::string, int> pos_idx_map =
      multibody::makeNameToPositionsMap(plant_control);
  std::map<std::string, int> vel_idx_map =
      multibody::makeNameToVelocitiesMap(plant_control);

  // Construct position index map
  stance_hip_roll_pos_map1_.insert(
      {left_right_support_fsm_states.at(0), pos_idx_map.at("hip_roll_left")});
  stance_hip_roll_pos_map1_.insert(
      {left_right_support_fsm_states.at(1), pos_idx_map.at("hip_roll_right")});
  stance_hip_pitch_pos_map1_.insert(
      {left_right_support_fsm_states.at(0), pos_idx_map.at("hip_pitch_left")});
  stance_hip_pitch_pos_map1_.insert(
      {left_right_support_fsm_states.at(1), pos_idx_map.at("hip_pitch_right")});
  stance_hip_yaw_pos_map1_.insert(
      {left_right_support_fsm_states.at(0), pos_idx_map.at("hip_yaw_left")});
  stance_hip_yaw_pos_map1_.insert(
      {left_right_support_fsm_states.at(1), pos_idx_map.at("hip_yaw_right")});
  swing_hip_yaw_pos_map1_.insert(
      {left_right_support_fsm_states.at(0), pos_idx_map.at("hip_yaw_right")});
  swing_hip_yaw_pos_map1_.insert(
      {left_right_support_fsm_states.at(1), pos_idx_map.at("hip_yaw_left")});
  // Construct velocity index map
  stance_hip_roll_vel_map1_.insert({left_right_support_fsm_states.at(0),
                                    vel_idx_map.at("hip_roll_leftdot")});
  stance_hip_roll_vel_map1_.insert({left_right_support_fsm_states.at(1),
                                    vel_idx_map.at("hip_roll_rightdot")});
  stance_hip_pitch_vel_map1_.insert({left_right_support_fsm_states.at(0),
                                     vel_idx_map.at("hip_pitch_leftdot")});
  stance_hip_pitch_vel_map1_.insert({left_right_support_fsm_states.at(1),
                                     vel_idx_map.at("hip_pitch_rightdot")});
  stance_hip_yaw_vel_map1_.insert(
      {left_right_support_fsm_states.at(0), vel_idx_map.at("hip_yaw_leftdot")});
  stance_hip_yaw_vel_map1_.insert({left_right_support_fsm_states.at(1),
                                   vel_idx_map.at("hip_yaw_rightdot")});
  swing_hip_yaw_vel_map1_.insert({left_right_support_fsm_states.at(0),
                                  vel_idx_map.at("hip_yaw_rightdot")});
  swing_hip_yaw_vel_map1_.insert(
      {left_right_support_fsm_states.at(1), vel_idx_map.at("hip_yaw_leftdot")});

  // Construct position index map
  stance_hip_roll_pos_map2_.insert({true, pos_idx_map.at("hip_roll_left")});
  stance_hip_roll_pos_map2_.insert({false, pos_idx_map.at("hip_roll_right")});
  stance_hip_pitch_pos_map2_.insert({true, pos_idx_map.at("hip_pitch_left")});
  stance_hip_pitch_pos_map2_.insert({false, pos_idx_map.at("hip_pitch_right")});
  stance_hip_yaw_pos_map2_.insert({true, pos_idx_map.at("hip_yaw_left")});
  stance_hip_yaw_pos_map2_.insert({false, pos_idx_map.at("hip_yaw_right")});
  swing_hip_yaw_pos_map2_.insert({true, pos_idx_map.at("hip_yaw_right")});
  swing_hip_yaw_pos_map2_.insert({false, pos_idx_map.at("hip_yaw_left")});
  // Construct velocity index map
  stance_hip_roll_vel_map2_.insert(
      {true, nq_ + vel_idx_map.at("hip_roll_leftdot")});
  stance_hip_roll_vel_map2_.insert(
      {false, nq_ + vel_idx_map.at("hip_roll_rightdot")});
  stance_hip_pitch_vel_map2_.insert(
      {true, nq_ + vel_idx_map.at("hip_pitch_leftdot")});
  stance_hip_pitch_vel_map2_.insert(
      {false, nq_ + vel_idx_map.at("hip_pitch_rightdot")});
  stance_hip_yaw_vel_map2_.insert(
      {true, nq_ + vel_idx_map.at("hip_yaw_leftdot")});
  stance_hip_yaw_vel_map2_.insert(
      {false, nq_ + vel_idx_map.at("hip_yaw_rightdot")});
  swing_hip_yaw_vel_map2_.insert(
      {true, nq_ + vel_idx_map.at("hip_yaw_rightdot")});
  swing_hip_yaw_vel_map2_.insert(
      {false, nq_ + vel_idx_map.at("hip_yaw_leftdot")});

  // Not sure why sometimes in the beginning of single support phase, the 0
  // desired traj was read (it evaluates the traj in previous double support
  // phase). As a quick fix, I just shift the time a bit.
  // TODO: fix this
  eps_hack_ = std::min(0.01, double_support_duration_);

  // Heuristic for spring model
  if ((plant_feedback.num_velocities() == plant_control.num_velocities()) &&
      !gains.use_hybrid_rom_mpc) {
    swing_foot_target_offset_x_ = 0;
  } else {
    swing_foot_target_offset_x_ = gains.swing_foot_target_offset_x;
  }
  final_foot_height_offset_for_right_leg_ =
      gains.final_foot_height_offset_for_right_leg;

  // // [Test sim gap] -- use trajopt's traj directly in OSC
  //  std::string dir = gains.dir_model + std::to_string(gains.model_iter) + "_"
  //  +
  //                    std::to_string(gains.sample_idx) + "_";
  //  cout << "dir = " << dir << endl;
  //  rom_pp_ = PiecewisePolynomial<double>::CubicHermite(
  //      readCSV(dir + "t_breaks0.csv").col(0), readCSV(dir +
  //      "y_samples0.csv"), readCSV(dir + "ydot_samples0.csv"));
  //
  //  n_mode_ = 2;
  //  x0_ = MatrixXd(nx_, n_mode_ + 1);
  //  x0_time_ = VectorXd(n_mode_ + 1);
  //  xf_ = MatrixXd(nx_, n_mode_);
  //  xf_time_ = VectorXd(n_mode_);
  //  stance_foot_ = VectorXd(n_mode_);
  //  VectorXd xpost = readCSV(dir + "x_samples1.csv").leftCols<1>();
  //  VectorXd xpre = readCSV(dir + "x_samples0.csv").rightCols<1>();
  //  double t_end = readCSV(dir + "t_breaks1.csv")(0, 0);
  //  x0_.col(0) = readCSV(dir + "x_samples0.csv").leftCols<1>();
  //  x0_.col(1) = xpost;
  //  x0_.col(2) << state_mirror.MirrorPos(xpost.head(nq_)),
  //      state_mirror.MirrorVel(xpost.tail(nv_));
  //  x0_time_ << 0, t_end, 2 * t_end;
  //  xf_.col(0) = xpre;
  //  xf_.col(1) << state_mirror.MirrorPos(xpre.head(nq_)),
  //      state_mirror.MirrorVel(xpre.tail(nv_));
  //  xf_time_ << t_end, 2 * t_end;
  //  stance_foot_ << 0, 1;  // left is 0, right is 1
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
  // RomPlannerTrajectory traj_data(*lcm_traj);

  // Construct cubic splines
  PiecewisePolynomial<double> pp =
      use_hybrid_rom_mpc_
          ? HybridRomPlannerTrajectory(*lcm_traj).ConstructPositionTrajectory()
          : RomPlannerTrajectory(*lcm_traj).ConstructPositionTrajectory();
  //  PiecewisePolynomial<double> pp = traj_data.ConstructPositionTrajectory();
  // // [Test sim gap] -- use trajopt's traj directly in OSC
  //  PiecewisePolynomial<double> pp = rom_pp_;

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
  // I added 1e-8 just in case any floating base error when rounding down double
  // to int
  double fsm_double = this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  int fsm_state = (fsm_double < -0.5) ? -1 : int(fsm_double + 1e-8);

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), fsm_state);
  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // when entering a new state which is in left_right_support_fsm_states
  if ((fsm_state != prev_fsm_state_) && is_single_support_phase) {
    prev_fsm_state_ = fsm_state;

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
    multibody::SetPositionsIfNew<double>(
        plant_feedback_, robot_output->GetPositions(), context_feedback_);

    // Swing foot position (Forward Kinematics) at touchdown
    auto swing_foot_pos_at_liftoff =
        discrete_state->get_mutable_vector(liftoff_swing_foot_pos_idx_)
            .get_mutable_value();
    const BodyPoint& swing_foot = swing_foot_map1_.at(fsm_state);
    plant_feedback_.CalcPointsPositions(
        *context_feedback_, swing_foot.second, swing_foot.first,
        plant_feedback_.world_frame(), &swing_foot_pos_at_liftoff);

    // Make swing foot pos relative to COM
    if (wrt_com_in_local_frame_) {
      swing_foot_pos_at_liftoff =
          view_frame_feedback_.CalcWorldToFrameRotation(plant_feedback_,
                                                        *context_feedback_) *
          (swing_foot_pos_at_liftoff -
           plant_feedback_.CalcCenterOfMassPositionInWorld(*context_feedback_));
    }

    // Stance hip joints
    discrete_state->get_mutable_vector(liftoff_stance_hip_pos_idx_)
            .get_mutable_value()
        << robot_output->GetPositions()(
               stance_hip_roll_pos_map1_.at(fsm_state)),
        robot_output->GetPositions()(stance_hip_pitch_pos_map1_.at(fsm_state)),
        robot_output->GetPositions()(stance_hip_yaw_pos_map1_.at(fsm_state));
    discrete_state->get_mutable_vector(liftoff_stance_hip_vel_idx_)
            .get_mutable_value()
        << robot_output->GetVelocities()(
               stance_hip_roll_vel_map1_.at(fsm_state)),
        robot_output->GetVelocities()(stance_hip_pitch_vel_map1_.at(fsm_state)),
        robot_output->GetVelocities()(stance_hip_yaw_vel_map1_.at(fsm_state));

    // Swing hip yaw joint
    discrete_state->get_mutable_vector(liftoff_swing_hip_pos_idx_)
            .get_mutable_value()
        << robot_output->GetPositions()(swing_hip_yaw_pos_map1_.at(fsm_state));
    discrete_state->get_mutable_vector(liftoff_swing_hip_vel_idx_)
            .get_mutable_value()
        << robot_output->GetVelocities()(swing_hip_yaw_vel_map1_.at(fsm_state));

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
  PiecewisePolynomial<double> pp;
  double current_time = context.get_time();

  if (use_hybrid_rom_mpc_) {
    HybridRomPlannerTrajectory traj_data(*lcm_traj);
    int n_mode = traj_data.GetNumModes();

    // Get foot steps (in global frame) and stance_foot
    VectorXd x0_time = traj_data.get_global_feet_pos_time().head(n_mode);
    VectorXd xf_time = traj_data.get_global_feet_pos_time().tail(n_mode);
    const MatrixXd& global_feet_pos = traj_data.get_global_feet_pos();
    const MatrixXd& global_com_pos = traj_data.get_global_com_pos();
    const VectorXd& stance_foot = traj_data.get_stance_foot();

    // Set feedback context
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
    multibody::SetPositionsIfNew<double>(
        plant_feedback_, robot_output->GetPositions(), context_feedback_);

    // Construct PP (concatenate the PP of each mode)
    // WARNING: we assume each mode in the planner is "single support" + "double
    // support"
    std::vector<double> T_waypoint = std::vector<double>(3, 0);
    std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
    Vector3d start_foot_pos;
    Vector3d end_foot_pos;
    bool init_step = true;
    bool left_stance = abs(stance_foot(0)) < 1e-12;
    for (int j = 0; j < n_mode; j++) {
      //      cout << "j = " << j << endl;

      // When the current time is bigger than the end time of the mode (this
      // could happen when the planner starts planning close the end of mode),
      // we skip. That is, we don't construct trajectories in the past, since we
      // are not going to use it.
      if (current_time < xf_time(j)) {
        if (xf_time(j) - double_support_duration_ > x0_time(j)) {
          // Set the time
          // T_waypoint.at(0) = (j == 0) ? lift_off_time_ : x0_time(j);
          T_waypoint.at(0) = (j == 0) ? xf_time(j) - single_support_duration_ -
                                            double_support_duration_
                                      : x0_time(j) - eps_hack_;
          T_waypoint.at(2) = xf_time(j) - double_support_duration_;
          T_waypoint.at(1) = (T_waypoint.at(0) + T_waypoint.at(2)) / 2;

          /// Foot x and y ///
          // Start pos
          if (init_step) {
            start_foot_pos =
                context.get_discrete_state(liftoff_swing_foot_pos_idx_)
                    .get_value();
            // We don't need `view_frame_feedback_` here, because it has been
            // done in the discreteUpdate above
          } else {
            start_foot_pos.head<2>() = global_feet_pos.col(j - 1);
            if (wrt_com_in_local_frame_) {
              start_foot_pos.head<2>() =
                  view_frame_feedback_
                      .CalcWorldToFrameRotation(plant_feedback_,
                                                *context_feedback_)
                      .topLeftCorner<2, 2>() *
                  (start_foot_pos.head<2>() - global_com_pos.col(j));
            }
            start_foot_pos(2) =
                context.get_discrete_state(liftoff_swing_foot_pos_idx_)
                    .get_value()(2);
          }
          // We don't need to shift `start_foot_pos` by
          // `swing_foot_target_offset_x_`, because we actually use the origin
          // for feedback value (same point as TrackingData's).
          Y.at(0) = start_foot_pos;

          // End pos
          end_foot_pos.head<2>() = global_feet_pos.col(j + 1);
          end_foot_pos(2) =
              context.get_discrete_state(liftoff_swing_foot_pos_idx_)
                  .get_value()(2);
          if (wrt_com_in_local_frame_) {
            end_foot_pos.head<2>() =
                view_frame_feedback_
                    .CalcWorldToFrameRotation(plant_feedback_,
                                              *context_feedback_)
                    .topLeftCorner<2, 2>() *
                (end_foot_pos.head<2>() - global_com_pos.col(j + 1));

            // Heuristics for spring model
            end_foot_pos(0) += swing_foot_target_offset_x_;
          }
          Y.at(2) = end_foot_pos;

          // Mid pos
          Y.at(1) = (Y.at(0) + Y.at(2)) / 2;

          /// Foot height ///
          if (init_step) {
            Y.at(1)(2) = start_foot_pos(2) + desired_mid_foot_height_;
            Y.at(2)(2) = start_foot_pos(2) + desired_final_foot_height_;
          } else {
            Y.at(1)(2) += desired_mid_foot_height_;
            Y.at(2)(2) += desired_final_foot_height_;
          }
          if (left_stance) {
            Y.at(2)(2) += final_foot_height_offset_for_right_leg_;
          }

          // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to
          // make the traj smooth at the mid point
          pp.ConcatenateInTime(
              PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
                  T_waypoint, Y, VectorXd::Zero(3), VectorXd::Zero(3)));
        }

        // Fill in the double support phase with a constant zero traj
        if (double_support_duration_ > 0) {
          VectorXd T_double_support(2);
          T_double_support << T_waypoint.at(2), xf_time(j) - eps_hack_;
          /*cout << "T_waypoint.at(2) = " << T_waypoint.at(2) << endl;
          cout << "xf_time(j) = " << xf_time(j) << endl;*/
          MatrixXd Y_double_support = MatrixXd::Zero(3, 2);
          pp.ConcatenateInTime(PiecewisePolynomial<double>::ZeroOrderHold(
              T_double_support, Y_double_support));
        }

        init_step = false;
      }

      left_stance = !left_stance;
    }

  } else {
    RomPlannerTrajectory traj_data(*lcm_traj);
    int n_mode = traj_data.GetNumModes();

    // Get states (in global frame) and stance_foot
    const MatrixXd& x0 = traj_data.get_x0();
    const VectorXd& x0_time = traj_data.get_x0_time();
    const MatrixXd& xf = traj_data.get_xf();
    const VectorXd& xf_time = traj_data.get_xf_time();
    const VectorXd& stance_foot = traj_data.get_stance_foot();
    DRAKE_DEMAND(xf_time(0) == x0_time(1));

    // // [Test sim gap] -- use trajopt's traj directly in OSC
    //  int n_mode = 2;
    //  const MatrixXd& x0 = x0_;
    //  const VectorXd& x0_time = x0_time_;
    //  const MatrixXd& xf = xf_;
    //  const VectorXd& xf_time = xf_time_;
    //  const VectorXd& stance_foot = stance_foot_;

    // Construct PP (concatenate the PP of each mode)
    // WARNING: we assume each mode in the planner is "single support" + "double
    // support"
    std::vector<double> T_waypoint = std::vector<double>(3, 0);
    std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
    Vector3d start_foot_pos;
    Vector3d end_foot_pos;
    bool init_step = true;
    bool left_stance = abs(stance_foot(0)) < 1e-12;
    for (int j = 0; j < n_mode; j++) {
      // When the current time is bigger than the end time of the mode (this
      // could happen when the planner starts planning close the end of mode),
      // we skip. That is, we don't construct trajectories in the past, since we
      // are not going to use it.
      if (current_time < xf_time(j)) {
        if (xf_time(j) - double_support_duration_ > x0_time(j)) {
          // Set the time
          // T_waypoint.at(0) = (j == 0) ? lift_off_time_ : x0_time(j);
          T_waypoint.at(0) = (j == 0) ? xf_time(j) - single_support_duration_ -
                                            double_support_duration_
                                      : x0_time(j) - eps_hack_;
          T_waypoint.at(2) = xf_time(j) - double_support_duration_;
          T_waypoint.at(1) = (T_waypoint.at(0) + T_waypoint.at(2)) / 2;

          // Start pos
          plant_control_.SetPositionsAndVelocities(context_control_.get(),
                                                   x0.col(j));
          if (init_step) {
            start_foot_pos =
                context.get_discrete_state(liftoff_swing_foot_pos_idx_)
                    .get_value();
          } else {
            plant_control_.CalcPointsPositions(
                *context_control_,
                left_right_foot_.at(left_stance ? 1 : 0).second,
                left_right_foot_.at(left_stance ? 1 : 0).first,
                plant_control_.world_frame(), &start_foot_pos);
            if (wrt_com_in_local_frame_) {
              start_foot_pos = view_frame_control_.CalcWorldToFrameRotation(
                                   plant_control_, *context_control_) *
                               (start_foot_pos -
                                plant_control_.CalcCenterOfMassPositionInWorld(
                                    *context_control_));
            }
          }
          Y.at(0) = start_foot_pos;
          // End pos
          plant_control_.SetPositionsAndVelocities(context_control_.get(),
                                                   xf.col(j));
          plant_control_.CalcPointsPositions(
              *context_control_,
              left_right_foot_.at(left_stance ? 1 : 0).second,
              left_right_foot_.at(left_stance ? 1 : 0).first,
              plant_control_.world_frame(), &end_foot_pos);
          if (wrt_com_in_local_frame_) {
            end_foot_pos =
                view_frame_control_.CalcWorldToFrameRotation(
                    plant_control_, *context_control_) *
                (end_foot_pos - plant_control_.CalcCenterOfMassPositionInWorld(
                                    *context_control_));

            // Heuristics for spring model
            end_foot_pos(0) += swing_foot_target_offset_x_;
          }
          Y.at(2) = end_foot_pos;
          // Mid pos
          Y.at(1) = (Y.at(0) + Y.at(2)) / 2;

          // Foot height
          if (init_step) {
            Y.at(1)(2) = start_foot_pos(2) + desired_mid_foot_height_;
            Y.at(2)(2) = start_foot_pos(2) + desired_final_foot_height_;
          } else {
            Y.at(1)(2) += desired_mid_foot_height_;
            Y.at(2)(2) += desired_final_foot_height_;
          }

          // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to
          // make the traj smooth at the mid point
          pp.ConcatenateInTime(
              PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
                  T_waypoint, Y, VectorXd::Zero(3), VectorXd::Zero(3)));
        }

        // Fill in the double support phase with a constant zero traj
        if (double_support_duration_ > 0) {
          VectorXd T_double_support(2);
          T_double_support << T_waypoint.at(2), xf_time(j) - eps_hack_;
          /*cout << "T_waypoint.at(2) = " << T_waypoint.at(2) << endl;
          cout << "xf_time(j) = " << xf_time(j) << endl;*/
          MatrixXd Y_double_support = MatrixXd::Zero(3, 2);
          pp.ConcatenateInTime(PiecewisePolynomial<double>::ZeroOrderHold(
              T_double_support, Y_double_support));
        }

        init_step = false;
      }

      left_stance = !left_stance;
    }

    // Testing -- trying to find the bug that the desired swing foot position
    // traj was evaluated 0 at start of mode sometimes in the OSC
    //  cout << "x0_time = " << x0_time.transpose() << endl;
    //  cout << "xf_time = " << xf_time.transpose() << endl;
    //  cout << "pp start = ";
    //  for (int i = 0; i < x0_time.size(); i++) {
    //    cout << pp.value(x0_time(i)).transpose() << endl;
    //  }
    //  cout << "pp end = ";
    //  for (int i = 0; i < xf_time.size(); i++) {
    //    cout << pp.value(xf_time(i)).transpose() << endl;
    //  }
    //  DRAKE_DEMAND(pp.value(x0_time(1)).norm() != 0);
    //  DRAKE_DEMAND(pp.value(xf_time(0)).norm() != 0);
  }

  // Assign traj
  *traj_casted = pp;
};

void SavedTrajReceiver::CalcStanceHipTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj
  auto* traj_casted = dynamic_cast<PiecewisePolynomial<double>*>(traj);

  // Read the lcm message
  auto lcm_traj = this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
      context, saved_traj_lcm_port_);

  // Check if traj is empty (if it's empty, it means we are haven't gotten the
  // traj from the planner yet)
  if (lcm_traj->saved_traj.num_trajectories == 0) {
    cout << "WARNING (CalcStanceHipRollTraj): trajectory size is 0!\n";
    *traj_casted = PiecewisePolynomial<double>(VectorXd::Zero(1));
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

  // // [Test sim gap] -- use trajopt's traj directly in OSC
  //  int n_mode = 2;
  //  const MatrixXd& x0 = x0_;
  //  const VectorXd& x0_time = x0_time_;
  //  const MatrixXd& xf = xf_;
  //  const VectorXd& xf_time = xf_time_;
  //  const VectorXd& stance_foot = stance_foot_;

  // Construct PP (concatenate the PP of each mode)
  // WARNING: we assume each mode in the planner is "single support" + "double
  // support"
  double current_time = context.get_time();

  PiecewisePolynomial<double> pp;
  std::vector<double> T_waypoint = std::vector<double>(2, 0);
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  std::vector<MatrixXd> Ydot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  Vector3d hip_pos;
  Vector3d hip_vel;
  bool init_step = true;
  bool left_stance = abs(stance_foot(0)) < 1e-12;
  for (int j = 0; j < n_mode; j++) {
    // When the current time is bigger than the end time of the mode (this could
    // happen when the planner starts planning close the end of mode), we skip
    if (current_time < xf_time(j)) {
      //   T_waypoint.at(0) = (j == 0) ? lift_off_time_ : x0_time(j);
      T_waypoint.at(0) = (j == 0) ? xf_time(j) - single_support_duration_ -
                                        double_support_duration_
                                  : x0_time(j);
      T_waypoint.at(1) = xf_time(j);

      // Start
      if (init_step) {
        // TODO: should I add a bound for the vel?
        hip_pos =
            context.get_discrete_state(liftoff_stance_hip_pos_idx_).get_value();
        hip_vel =
            context.get_discrete_state(liftoff_stance_hip_vel_idx_).get_value();
      } else {
        hip_pos << x0.col(j)(stance_hip_roll_pos_map2_.at(left_stance)),
            x0.col(j)(stance_hip_pitch_pos_map2_.at(left_stance)),
            x0.col(j)(stance_hip_yaw_pos_map2_.at(left_stance));
        hip_vel << x0.col(j)(stance_hip_roll_vel_map2_.at(left_stance)),
            x0.col(j)(stance_hip_pitch_vel_map2_.at(left_stance)),
            x0.col(j)(stance_hip_yaw_vel_map2_.at(left_stance));
      }
      Y.at(0) = hip_pos;
      Ydot.at(0) = hip_vel;
      // End
      hip_pos << xf.col(j)(stance_hip_roll_pos_map2_.at(left_stance)),
          xf.col(j)(stance_hip_pitch_pos_map2_.at(left_stance)),
          xf.col(j)(stance_hip_yaw_pos_map2_.at(left_stance));
      hip_vel << xf.col(j)(stance_hip_roll_vel_map2_.at(left_stance)),
          xf.col(j)(stance_hip_pitch_vel_map2_.at(left_stance)),
          xf.col(j)(stance_hip_yaw_vel_map2_.at(left_stance));
      Y.at(1) = hip_pos;
      Ydot.at(1) = hip_vel;

      // Create segment
      pp.ConcatenateInTime(
          PiecewisePolynomial<double>::CubicHermite(T_waypoint, Y, Ydot));

      init_step = false;
    }

    left_stance = !left_stance;
  }

  // Assign traj
  *traj_casted = pp;
};

void SavedTrajReceiver::CalcSwingHipTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj
  auto* traj_casted = dynamic_cast<PiecewisePolynomial<double>*>(traj);

  // Read the lcm message
  auto lcm_traj = this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
      context, saved_traj_lcm_port_);

  // Check if traj is empty (if it's empty, it means we are haven't gotten the
  // traj from the planner yet)
  if (lcm_traj->saved_traj.num_trajectories == 0) {
    cout << "WARNING (CalcSwingHipTraj): trajectory size is 0!\n";
    *traj_casted = PiecewisePolynomial<double>(VectorXd::Zero(1));
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

  //  // [Test sim gap] -- use trajopt's traj directly in OSC
  //  int n_mode = 2;
  //  const MatrixXd& x0 = x0_;
  //  const VectorXd& x0_time = x0_time_;
  //  const MatrixXd& xf = xf_;
  //  const VectorXd& xf_time = xf_time_;
  //  const VectorXd& stance_foot = stance_foot_;

  // Construct PP (concatenate the PP of each mode)
  // WARNING: we assume each mode in the planner is "single support" + "double
  // support"
  double current_time = context.get_time();

  PiecewisePolynomial<double> pp;
  std::vector<double> T_waypoint = std::vector<double>(2, 0);
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(1, 1));
  std::vector<MatrixXd> Ydot(T_waypoint.size(), MatrixXd::Zero(1, 1));
  VectorXd hip_pos(1);
  VectorXd hip_vel(1);
  bool init_step = true;
  bool left_stance = abs(stance_foot(0)) < 1e-12;
  for (int j = 0; j < n_mode; j++) {
    // When the current time is bigger than the end time of the mode (this could
    // happen when the planner starts planning close the end of mode), we skip
    if (current_time < xf_time(j)) {
      //   T_waypoint.at(0) = (j == 0) ? lift_off_time_ : x0_time(j);
      T_waypoint.at(0) = (j == 0) ? xf_time(j) - single_support_duration_ -
                                        double_support_duration_
                                  : x0_time(j);
      // TODO (yminchen): the end time should actually be `xf_time(j) -
      // double_support_duration_`
      T_waypoint.at(1) = xf_time(j);

      // Start
      if (init_step) {
        // TODO: should I add a bound for the vel?
        hip_pos =
            context.get_discrete_state(liftoff_swing_hip_pos_idx_).get_value();
        hip_vel =
            context.get_discrete_state(liftoff_swing_hip_vel_idx_).get_value();
      } else {
        hip_pos << x0.col(j)(swing_hip_yaw_pos_map2_.at(left_stance));
        hip_vel << x0.col(j)(swing_hip_yaw_vel_map2_.at(left_stance));
      }
      Y.at(0) = hip_pos;
      Ydot.at(0) = hip_vel;
      // End
      hip_pos << xf.col(j)(swing_hip_yaw_pos_map2_.at(left_stance));
      hip_vel << xf.col(j)(swing_hip_yaw_vel_map2_.at(left_stance));
      Y.at(1) = hip_pos;
      Ydot.at(1) = hip_vel;

      // Create segment
      pp.ConcatenateInTime(
          PiecewisePolynomial<double>::CubicHermite(T_waypoint, Y, Ydot));

      init_step = false;
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
