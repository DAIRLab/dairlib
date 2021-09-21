#include "systems/controllers/lipm_traj_gen.h"

#include <math.h>

#include <string>
#include <fstream>

#include <drake/math/saturate.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {

LIPMTrajGenerator::LIPMTrajGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context,
    double desired_com_height, const vector<int>& unordered_fsm_states,
    const vector<double>& unordered_state_durations,
    const vector<vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>>&
        contact_points_in_each_state,
    bool use_CoM)
    : plant_(plant),
      context_(context),
      desired_com_height_(desired_com_height),
      unordered_fsm_states_(unordered_fsm_states),
      unordered_state_durations_(unordered_state_durations),
      contact_points_in_each_state_(contact_points_in_each_state),
      world_(plant_.world_frame()),
      use_com_(use_CoM) {
  if (use_CoM) {
    this->set_name("lipm_traj");
  } else {
    this->set_name("pelvis_traj");
  }

  // Checking vector dimension
  DRAKE_DEMAND(unordered_fsm_states.size() == unordered_state_durations.size());
  DRAKE_DEMAND(unordered_fsm_states.size() ==
               contact_points_in_each_state.size());

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  touchdown_time_port_ =
      this->DeclareVectorInputPort("t_touchdown", BasicVector<double>(1))
          .get_index();

  // Provide an instance to allocate the memory first (for the output)
  ExponentialPlusPiecewisePolynomial<double> exp;
  drake::trajectories::Trajectory<double>& traj_inst = exp;
  output_port_lipm_from_current_ =
      this->DeclareAbstractOutputPort("lipm_xyz_from_current", traj_inst,
                                      &LIPMTrajGenerator::CalcTrajFromCurrent)
          .get_index();
  output_port_lipm_from_touchdown_ =
      this->DeclareAbstractOutputPort("lipm_xyz_from_touchdown", traj_inst,
                                      &LIPMTrajGenerator::CalcTrajFromTouchdown)
          .get_index();

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&LIPMTrajGenerator::DiscreteVariableUpdate);
  // The last FSM event time
  prev_touchdown_time_idx_ = this->DeclareDiscreteState(-1 * VectorXd::Ones(1));
  // The stance foot position in the beginning of the swing phase
  stance_foot_pos_idx_ = this->DeclareDiscreteState(3);
  // COM state at touchdown
  touchdown_com_pos_idx_ = this->DeclareDiscreteState(3);
  touchdown_com_vel_idx_ = this->DeclareDiscreteState(3);
  prev_fsm_idx_ = this->DeclareDiscreteState(-1 * VectorXd::Ones(1));
}

EventStatus LIPMTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in previous touchdown time
  auto prev_touchdown_time =
      discrete_state->get_mutable_vector(prev_touchdown_time_idx_)
          .get_mutable_value();
  double touchdown_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);

  // Read in finite state machine
  auto fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()(0);

  // when entering a new stance phase
  if (fsm_state != discrete_state->get_vector(prev_fsm_idx_).GetAtIndex(0)) {
    prev_touchdown_time << touchdown_time;

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
    VectorXd v = robot_output->GetVelocities();
    multibody::SetPositionsAndVelocitiesIfNew<double>(
        plant_, robot_output->GetState(), context_);

    // Find fsm_state in unordered_fsm_states_
    auto it = find(unordered_fsm_states_.begin(), unordered_fsm_states_.end(),
                   fsm_state);
    int mode_index = std::distance(unordered_fsm_states_.begin(), it);
    if (it == unordered_fsm_states_.end()) {
      std::cerr << "WARNING: fsm state number " << fsm_state
                << " doesn't exist in LIPMTrajGenerator\n";
      mode_index = 0;
    }

    // Stance foot position (Forward Kinematics)
    // Take the average of all the points
    Vector3d stance_foot_pos = Vector3d::Zero();
    for (const auto& j : contact_points_in_each_state_[mode_index]) {
      Vector3d position;
      plant_.CalcPointsPositions(*context_, j.second, j.first, world_,
                                 &position);
      stance_foot_pos += position;
    }
    stance_foot_pos /= contact_points_in_each_state_[mode_index].size();

    // Get center of mass position and velocity
    Vector3d CoM;
    MatrixXd J(3, plant_.num_velocities());
    if (use_com_) {
      CoM = plant_.CalcCenterOfMassPositionInWorld(*context_);
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *context_, JacobianWrtVariable::kV, world_, world_, &J);
    } else {
      plant_.CalcPointsPositions(*context_,
                                 plant_.GetBodyByName("pelvis").body_frame(),
                                 VectorXd::Zero(3), world_, &CoM);
      plant_.CalcJacobianTranslationalVelocity(
          *context_, JacobianWrtVariable::kV,
          plant_.GetBodyByName("pelvis").body_frame(), VectorXd::Zero(3),
          world_, world_, &J);
    }
    Vector3d dCoM = J * v;

    discrete_state->get_mutable_vector(stance_foot_pos_idx_).get_mutable_value()
        << stance_foot_pos;
    discrete_state->get_mutable_vector(touchdown_com_pos_idx_)
            .get_mutable_value()
        << CoM;
    discrete_state->get_mutable_vector(touchdown_com_vel_idx_)
            .get_mutable_value()
        << dCoM;
  }

  discrete_state->get_mutable_vector(prev_fsm_idx_).GetAtIndex(0) = fsm_state;

  return EventStatus::Succeeded();
}

ExponentialPlusPiecewisePolynomial<double> LIPMTrajGenerator::ConstructLipmTraj(
    const VectorXd& CoM, const VectorXd& dCoM, const VectorXd& stance_foot_pos,
    double start_time, double end_time_of_this_fsm_state) const {
  // Get CoM_wrt_foot for LIPM
  double CoM_wrt_foot_x = CoM(0) - stance_foot_pos(0);
  double CoM_wrt_foot_y = CoM(1) - stance_foot_pos(1);
  double CoM_wrt_foot_z = (CoM(2) - stance_foot_pos(2));
  double dCoM_wrt_foot_x = dCoM(0);
  double dCoM_wrt_foot_y = dCoM(1);
  DRAKE_DEMAND(CoM_wrt_foot_z > 0);

  // create a 3D one-segment polynomial for ExponentialPlusPiecewisePolynomial
  // Note that the start time in T_waypoint_com is also used by
  // ExponentialPlusPiecewisePolynomial.
  vector<double> T_waypoint_com = {start_time, end_time_of_this_fsm_state};

  vector<MatrixXd> Y(T_waypoint_com.size(), MatrixXd::Zero(3, 1));
  Y[0](0, 0) = stance_foot_pos(0);
  Y[1](0, 0) = stance_foot_pos(0);
  Y[0](1, 0) = stance_foot_pos(1);
  Y[1](1, 0) = stance_foot_pos(1);
  // We add stance_foot_pos(2) to desired COM height to account for state
  // drifting
  double max_height_diff_per_step = 0.05;
  double final_height = drake::math::saturate(
      desired_com_height_ + stance_foot_pos(2),
      CoM(2) - max_height_diff_per_step, CoM(2) + max_height_diff_per_step);
  //  double final_height = desired_com_height_ + stance_foot_pos(2);
  Y[0](2, 0) = final_height;
  Y[1](2, 0) = final_height;

  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);

  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          T_waypoint_com, Y, Y_dot_start, Y_dot_end);

  // Dynamics of LIPM
  // ddy = 9.81/CoM_wrt_foot_z*y, which has an analytical solution.
  // Let omega^2 = 9.81/CoM_wrt_foot_z.
  // Let y0 and dy0 be the intial position and velocity. Then the solution is
  //   y = k_1 * exp(w*t) + k_2 * exp(-w*t)
  // where k_1 = (y0 + dy0/w)/2
  //       k_2 = (y0 - dy0/w)/2.
  // double omega = sqrt(9.81 / (final_height - stance_foot_pos(2)));
  double omega = sqrt(9.81 / CoM_wrt_foot_z);
  double k1x = 0.5 * (CoM_wrt_foot_x + dCoM_wrt_foot_x / omega);
  double k2x = 0.5 * (CoM_wrt_foot_x - dCoM_wrt_foot_x / omega);
  double k1y = 0.5 * (CoM_wrt_foot_y + dCoM_wrt_foot_y / omega);
  double k2y = 0.5 * (CoM_wrt_foot_y - dCoM_wrt_foot_y / omega);

  //  cout << "omega = " << omega << endl;

  // Sum of two exponential + one-segment 3D polynomial
  MatrixXd K = MatrixXd::Zero(3, 2);
  MatrixXd A = MatrixXd::Zero(2, 2);
  MatrixXd alpha = MatrixXd::Zero(2, 1);
  K << k1x, k2x, k1y, k2y, 0, 0;
  A << omega, 0, 0, -omega;
  alpha << 1, 1;

  return ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, pp_part);
}

void Print3dVector(VectorXd input) {
  cout << "np.array([" << input(0) << ", " << input(1) << ", " << input(2)
       << "])\n";
}

void LIPMTrajGenerator::CalcTrajFromCurrent(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd v = robot_output->GetVelocities();
  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  // Read in finite state machine switch time
  VectorXd prev_event_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value();

  // Find fsm_state in unordered_fsm_states_
  auto it = find(unordered_fsm_states_.begin(), unordered_fsm_states_.end(),
                 int(fsm_state(0)));
  int mode_index = std::distance(unordered_fsm_states_.begin(), it);
  if (it == unordered_fsm_states_.end()) {
    std::cerr << "WARNING: fsm state number " << fsm_state(0)
              << " doesn't exist in LIPMTrajGenerator\n";
    mode_index = 0;
  }

  // Get time
  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp;

  double end_time =
      prev_event_time(0) + unordered_state_durations_[mode_index];
  // Ensure "current_time < end_time" to avoid error in
  // creating trajectory.
  start_time = drake::math::saturate(
      start_time, -std::numeric_limits<double>::infinity(), end_time - 0.001);

  VectorXd q = robot_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  // Get center of mass position and velocity
  Vector3d CoM;
  MatrixXd J(3, plant_.num_velocities());
  if (use_com_) {
    CoM = plant_.CalcCenterOfMassPositionInWorld(*context_);
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, world_, world_, &J);
  } else {
    plant_.CalcPointsPositions(*context_,
                               plant_.GetBodyByName("pelvis").body_frame(),
                               VectorXd::Zero(3), world_, &CoM);
    plant_.CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kV,
        plant_.GetBodyByName("pelvis").body_frame(), VectorXd::Zero(3), world_,
        world_, &J);
  }
  Vector3d dCoM = J * v;

  // Stance foot position (Forward Kinematics)
  // Take the average of all the points
  Vector3d stance_foot_pos = Vector3d::Zero();
  for (const auto& stance_foot : contact_points_in_each_state_[mode_index]) {
    Vector3d position;
    plant_.CalcPointsPositions(*context_, stance_foot.second, stance_foot.first,
                               world_, &position);
    stance_foot_pos += position;
  }
  stance_foot_pos /= contact_points_in_each_state_[mode_index].size();

  // Assign traj
  auto exp_pp_traj = (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
      ExponentialPlusPiecewisePolynomial<double>*>(traj);
  *exp_pp_traj = ConstructLipmTraj(CoM, dCoM, stance_foot_pos, start_time, end_time);

  /*cout << "start_time = " << start_time << endl;
  cout << "end_time = " << end_time << endl;
  cout << "stance_foot_pos = ";
  Print3dVector(stance_foot_pos);
  cout << "CoM_wrt_stance_foot_pos = ";
  Print3dVector((CoM - stance_foot_pos));
  cout << "dCoM = ";
  Print3dVector(dCoM);
  cout << "desired_com_height_ = " << desired_com_height_ << endl;
  int n = 4;
  cout << "N = " << n << endl;

  VectorXd times(n);
  for (int i = 0; i < n; i++) {
    times(i) = start_time + i * (end_time - start_time) / (n - 1);
  }
  cout << "\"\"\"\n";
  cout << "\ntimes = " << times.transpose() << endl;

  cout << "\ncom along traj = \n";
  for (int i = 0; i < n; i++) {
    double t = times(i);
    VectorXd vec = exp_pp_traj->value(t);
    cout << vec(0) << ", " << vec(1) << ", " << vec(2) << endl;
  }
  cout << "\ncomdot along traj = \n";
  for (int i = 0; i < n; i++) {
    double t = times(i);
    VectorXd vec = exp_pp_traj->MakeDerivative(1)->value(t);
    cout << vec(0) << ", " << vec(1) << ", " << vec(2) << endl;
  }
  cout << "\"\"\"\n";
  cout << endl;*/

  /// Test centroidal momentum
  /*drake::Vector3<double> p_WC =
      plant_.CalcCenterOfMassPositionInWorld(*context_);
  drake::multibody::SpatialMomentum<double> h_WC_eval =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, p_WC);
  auto angular_momentum_wrt_com = h_WC_eval.rotational();
  std::ofstream outfile;
  outfile.open("../debug_centroidal_momentum.txt", std::ios_base::app);
  outfile << robot_output->get_timestamp() << ", ";
  for (int i = 0; i < angular_momentum_wrt_com.size(); i++) {
    outfile << angular_momentum_wrt_com(i);
    if (i == angular_momentum_wrt_com.size() - 1) {
      outfile << "\n";
    } else {
      outfile << ", ";
    }
  }*/
}
void LIPMTrajGenerator::CalcTrajFromTouchdown(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();
  // Read in finite state machine switch time
  VectorXd prev_event_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value();

  // TODO(yangwill): move this in a function or make it shorter
  // Find fsm_state in unordered_fsm_states_
  auto it = find(unordered_fsm_states_.begin(), unordered_fsm_states_.end(),
                 int(fsm_state(0)));
  int mode_index = std::distance(unordered_fsm_states_.begin(), it);
  if (it == unordered_fsm_states_.end()) {
    cout << "WARNING: fsm state number " << fsm_state(0)
         << " doesn't exist in LIPMTrajGenerator\n";
    mode_index = 0;
  }

  double end_time_of_this_fsm_state =
      prev_event_time(0) + unordered_state_durations_[mode_index];

  // Get center of mass position and velocity
  const auto CoM_at_touchdown =
      context.get_discrete_state(touchdown_com_pos_idx_).get_value();
  const auto dCoM_at_touchdown =
      context.get_discrete_state(touchdown_com_vel_idx_).get_value();

  // Stance foot position
  const auto stance_foot_pos_at_touchdown =
      context.get_discrete_state(stance_foot_pos_idx_).get_value();

  double prev_touchdown_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);

  // Assign traj
  auto exp_pp_traj = (ExponentialPlusPiecewisePolynomial<double>*)dynamic_cast<
      ExponentialPlusPiecewisePolynomial<double>*>(traj);
  *exp_pp_traj = ConstructLipmTraj(
      CoM_at_touchdown, dCoM_at_touchdown, stance_foot_pos_at_touchdown,
      prev_touchdown_time, end_time_of_this_fsm_state);
}

}  // namespace systems
}  // namespace dairlib
