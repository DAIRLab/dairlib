#include "hybrid_lqr.h"
#include <drake/lcmt_contact_results_for_viz.hpp>
#include "multibody/multibody_utils.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

using dairlib::multibody::ContactInfo;
using dairlib::multibody::ContactToolkit;
using dairlib::multibody::createContext;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::initializeAutoDiff;
using drake::multibody::Body;
using drake::multibody::JointActuatorIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DenseOutput;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::InitialValueProblem;
using drake::systems::controllers::LinearQuadraticRegulator;
using drake::systems::controllers::LinearQuadraticRegulatorResult;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib::systems {

using drake::AutoDiffXd;
using drake::trajectories::Trajectory;
using Eigen::AutoDiffScalar;
using Eigen::HouseholderQR;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::ContactInfo;
using std::cout;
using std::endl;
using std::make_unique;
using std::shared_ptr;
using std::vector;

MatrixXd generate_state_input_matrix(drake::systems::DenseOutput<double>&,
                                     VectorXd& times);

HybridLQRController::HybridLQRController(
    const MultibodyPlant<double>& plant,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const vector<ContactInfo<double>>& contact_info,
    const vector<ContactInfo<AutoDiffXd>>& contact_info_ad, const MatrixXd& Q,
    const MatrixXd& R, const MatrixXd& Qf,
    const vector<shared_ptr<PiecewisePolynomial<double>>>& state_traj,
    const vector<shared_ptr<PiecewisePolynomial<double>>>& input_traj,
    const vector<double>& impact_times, string folder_path, bool naive_approach,
    bool using_min_coords, bool recalculateP, bool recalculateL)
    : plant_(plant),
      plant_ad_(plant_ad),
      contact_info_(contact_info),
      Q_(Q),
      R_(R),
      Qf_(Qf),
      state_trajs_(state_traj),
      input_trajs_(input_traj),
      impact_times_(impact_times),
      folder_path_(folder_path),
      naive_approach_(naive_approach),
      using_min_coords_(using_min_coords),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_x_(n_q_ + n_v_),
      n_u_(plant.num_actuators()),
      n_c_(2),
      n_d_(n_x_ - 2 * n_c_),
      num_modes_(contact_info.size()) {
  //  DRAKE_DEMAND(state_trajs_.end_time() == input_trajs_.end_time());
  DRAKE_DEMAND(n_q_ == n_v_);
  DRAKE_DEMAND(impact_times.size() == 2 * num_modes_);

  TXZ_ << 1, 0, 0, 0, 0, 1;

  // Not as simple as just reversing the vector. Need to change the times as
  // well
  impact_times_rev_ = std::vector<double>(impact_times_.size());
  for (int i = 0; i < impact_times_.size(); ++i) {  // iterate in reverse
    impact_times_rev_[i] = getReverseTime(impact_times_[i]);
  }
  std::reverse(impact_times_rev_.begin(), impact_times_rev_.end());
  //  std::reverse_copy(std::begin(impact_times_), std::end(impact_times_),
  //                    std::begin(impact_times_rev_));
  contact_info_rev_ = std::vector<ContactInfo<double>>(num_modes_);
  contact_info_ad_ = std::vector<ContactInfo<AutoDiffXd>>(num_modes_);
  std::reverse_copy(std::begin(contact_info_), std::end(contact_info_),
      std::begin(contact_info_rev_));
  std::reverse_copy(std::begin(contact_info_ad), std::end(contact_info_ad),
      std::begin(contact_info_ad_));

  // Declare all system ports
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
              plant.num_velocities(),
              plant.num_actuators()))
          .get_index();
  control_output_port_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(n_u_),
              &HybridLQRController::CalcControl)
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  contact_port_ = this->DeclareAbstractInputPort(
                          "lcmt_contact_info",
                          drake::Value<drake::lcmt_contact_results_for_viz>{})
                      .get_index();

  // Get input limits
  VectorXd u_min(n_u_);
  VectorXd u_max(n_u_);
  for (JointActuatorIndex i(0); i < n_u_; i++) {
    u_min(i) = -plant.get_joint_actuator(i).effort_limit();
    u_max(i) = plant.get_joint_actuator(i).effort_limit();
  }
  u_min_ = u_min;
  u_max_ = u_max;

  l_t_ = std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>>(
      num_modes_);
  p_t_ = std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>>(
      num_modes_);

  MatrixXd S_f;
  if (using_min_coords_) {
    if (recalculateP) {
      cout << "Calculating minimal coord basis and saving\n";
      calcMinimalCoordBasis();
    }
    const LcmTrajectory& P_traj = LcmTrajectory(folder_path_ + "P_traj");

    const LcmTrajectory::Trajectory& P_mode0 = P_traj.getTrajectory("P0");
    const LcmTrajectory::Trajectory& P_mode1 = P_traj.getTrajectory("P1");
    const LcmTrajectory::Trajectory& P_mode2 = P_traj.getTrajectory("P2");

    p_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
        P_mode0.time_vector, P_mode0.datapoints));
    p_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
        P_mode1.time_vector, P_mode1.datapoints));
    p_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
        P_mode2.time_vector, P_mode2.datapoints));
    MatrixXd P = getMinimalCoordBasis(0.0, 0);
    S_f = P * Qf_ * P.transpose();
  } else {
    S_f = Qf_;
  }

  // Set the filepath for the different options
  l_traj_filepath_ = folder_path_ + "L_traj";
  if (using_min_coords_) l_traj_filepath_ = l_traj_filepath_ + "_min";
  if (!naive_approach_) l_traj_filepath_ = l_traj_filepath_ + "_adjusted";

  if (recalculateL) {
    calcCostToGo(S_f);
  }

  std::cout << l_traj_filepath_ << std::endl;
  const LcmTrajectory& L_traj = LcmTrajectory(l_traj_filepath_);

  // Keep in mind that the L trajectory is backwards in time
  const LcmTrajectory::Trajectory& L_mode0 = L_traj.getTrajectory("L2");
  const LcmTrajectory::Trajectory& L_mode1 = L_traj.getTrajectory("L1");
  const LcmTrajectory::Trajectory& L_mode2 = L_traj.getTrajectory("L0");

  l_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
      L_mode0.time_vector, L_mode0.datapoints));
  l_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
      L_mode1.time_vector, L_mode1.datapoints));
  l_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
      L_mode2.time_vector, L_mode2.datapoints));
}

void HybridLQRController::calcLinearizedDynamics(double t, int contact_mode,
                                                 MatrixXd* A_linear,
                                                 MatrixXd* B_linear) {
  DRAKE_DEMAND(A_linear->cols() == A_linear->rows());
  DRAKE_DEMAND(A_linear->cols() == n_x_);
  double t_rev = getReverseTime(t);
  int rev_mode = getReverseMode(contact_mode);
  VectorXd state = state_trajs_[rev_mode]->value(t_rev);
  VectorXd input = input_trajs_[rev_mode]->value(t_rev);

  VectorXd xu(n_x_ + n_u_);
  xu << state, input;
  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);  // first segment
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);  // middle segment

  std::unique_ptr<drake::systems::Context<AutoDiffXd>> context =
      createContext(plant_ad_, x_autodiff, u_autodiff);

  AutoDiffVecXd pt_cast = VectorXd::Zero(3);  // no offset from foot position
  AutoDiffVecXd pt_transform(3);
  MatrixX<AutoDiffXd> J3d(3, n_v_);
  const drake::multibody::Frame<AutoDiffXd>& world = plant_ad_.world_frame();
  const drake::multibody::Frame<AutoDiffXd>& contact_frame =
      *contact_info_ad_[contact_mode].frameA[0];

  plant_ad_.CalcPointsPositions(*context, contact_frame, pt_cast, world,
      &pt_transform);
  plant_ad_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, contact_frame,
      pt_cast, world, world, &J3d);
  MatrixX<AutoDiffXd> J3d_times_v =
      plant_ad_
          .CalcBiasForJacobianSpatialVelocity(
              *context, drake::multibody::JacobianWrtVariable::kV,
              contact_frame, pt_cast, world, world)
          .tail(3);

  MatrixX<AutoDiffXd> J = TXZ_ * J3d;
  AutoDiffVecXd JdotV = TXZ_ * J3d_times_v;
  MatrixX<AutoDiffXd> M(n_v_, n_v_);
  plant_ad_.CalcMassMatrixViaInverseDynamics(*context, &M);
  MatrixX<AutoDiffXd> B = plant_.MakeActuationMatrix();
  AutoDiffVecXd C(n_v_);
  plant_ad_.CalcBiasTerm(*context, &C);
  AutoDiffVecXd g(n_v_);
  g = plant_ad_.CalcGravityGeneralizedForces(*context);
  AutoDiffVecXd lambda(n_c_);
  MatrixX<AutoDiffXd> M_inv = M.inverse();
  AutoDiffVecXd rhs(n_v_ + n_c_);
  rhs << B * u_autodiff - C + g, -JdotV;
  MatrixX<AutoDiffXd> lhs(n_v_ + n_c_, n_v_ + n_c_);
  lhs << M, -J.transpose(), J, MatrixXd::Zero(n_c_, n_c_);
  AutoDiffVecXd qddot_lambda = lhs.inverse() * rhs;
  AutoDiffVecXd qddot = qddot_lambda.head(n_v_);
  AutoDiffVecXd xdot(n_x_);

  xdot << x_autodiff.tail(n_v_), qddot;
  MatrixXd AB = drake::math::autoDiffToGradientMatrix(xdot);

  *A_linear << AB.leftCols(n_x_);
  *B_linear << AB.block(0, n_x_, AB.rows(), n_u_);
}

MatrixXd HybridLQRController::calcJumpMap(MatrixXd& S_pre, int contact_mode,
                                          double t) {
  double t_rev = getReverseTime(t);
  int rev_mode = getReverseMode(contact_mode + 1);
  //  int rev_mode = getReverseMode(contact_mode);
  VectorXd x_pre = state_trajs_[rev_mode]->value(t_rev);
  VectorXd u_pre = input_trajs_[rev_mode]->value(t_rev);
  std::unique_ptr<drake::systems::Context<double>> context =
      createContext(plant_, x_pre, u_pre);
  const drake::multibody::Frame<double>& world = plant_.world_frame();

  MatrixXd M(n_v_, n_v_);
  plant_.CalcMassMatrixViaInverseDynamics(*context, &M);
  MatrixXd B = plant_.MakeActuationMatrix();
  VectorXd g = plant_.CalcGravityGeneralizedForces(*context);
  MatrixXd M_inv = M.inverse();

  VectorXd C(n_v_);
  plant_.CalcBiasTerm(*context, &C);

  const drake::multibody::Frame<double>& contact_frame_pre =
      *contact_info_rev_[contact_mode].frameA[0];
  VectorXd pt_cast = VectorXd::Zero(3);
  MatrixXd J_pre_3d(3, n_v_);

  plant_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, contact_frame_pre,
      pt_cast, world, world, &J_pre_3d);

  //  std::cout << "Pre-impact contact frame: " << contact_frame_pre.name()
  //            << std::endl;
  MatrixXd JdotV_3d =
      plant_
          .CalcBiasForJacobianSpatialVelocity(
              *context, drake::multibody::JacobianWrtVariable::kV,
              contact_frame_pre, pt_cast, world, world)
          .tail(3);
  MatrixXd J_pre = TXZ_ * J_pre_3d;
  VectorXd JdotV = TXZ_ * JdotV_3d;
  VectorXd rhs(n_v_ + n_c_);
  rhs << B * u_pre - C + g, -JdotV;
  MatrixXd lhs(n_v_ + n_c_, n_v_ + n_c_);
  lhs << M, -J_pre.transpose(), J_pre, MatrixXd::Zero(n_c_, n_c_);
  VectorXd qddot_lambda = lhs.inverse() * rhs;
  VectorXd qddot = qddot_lambda.head(n_v_);
  //  VectorXd xdot_pre(n_x_);
  //  xdot_pre << x_pre.tail(n_v_), qddot;  // assuming v is qdot
  VectorXd xdot_pre = state_trajs_[rev_mode]->derivative(1).value(t_rev);

  // Reinitializing contact toolkit for new contact mode
  // Also reinitializing context
  int contact_mode_post = contact_mode;
  int rev_mode_post = getReverseMode(contact_mode_post);
  VectorXd x_post = state_trajs_[rev_mode_post]->value(t_rev);
  VectorXd u_post = input_trajs_[rev_mode_post]->value(t_rev);
  context = createContext(plant_, x_post, u_post);

  const drake::multibody::Frame<double>& new_contact_frame =
      *contact_info_rev_[contact_mode_post + 1].frameA[0];

  //  std::cout << "Post-impact contact frame: " << new_contact_frame.name()
  //            << std::endl;
  MatrixXd J_post_3d(3, n_v_);

  plant_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, new_contact_frame,
      pt_cast, world, world, &J_post_3d);
  MatrixXd JdotV_post_3d =
      plant_
          .CalcBiasForJacobianSpatialVelocity(
              *context, drake::multibody::JacobianWrtVariable::kV,
              new_contact_frame, pt_cast, world, world)
          .tail(3);
  MatrixXd J_post = TXZ_ * J_post_3d;
  VectorXd JdotV_post = TXZ_ * JdotV_post_3d;
  rhs << B * u_post - C + g, -JdotV_post;
  lhs << M, -J_post.transpose(), J_post, MatrixXd::Zero(n_c_, n_c_);
  qddot_lambda = lhs.inverse() * rhs;
  qddot = qddot_lambda.head(n_v_);
  //  VectorXd xdot_post(n_x_);
  //  xdot_post << x_post.tail(n_v_), qddot;  // assuming v is qdot
  VectorXd xdot_post = state_trajs_[rev_mode_post]->derivative(1).value(t_rev);

  // Now calculate H
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  MatrixXd R = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd Dx_R = MatrixXd::Zero(n_x_, n_x_);
  calcLinearResetMap(t, contact_mode, &R, &Dx_R);
  VectorXd D1_g(n_x_);

  D1_g
      << J_pre.row(1).transpose(), VectorXd::Zero(n_q_);  // dphi/dq, dphi/dqdot
  VectorXd J_delta = Dx_R * xdot_pre;
  double J_g = (J_pre.row(1) * xdot_pre.head(n_q_));

  MatrixXd H = MatrixXd::Zero(n_x_, n_x_);

  H << (R + ((xdot_post - xdot_pre - J_delta) / J_g) * D1_g.transpose());
  // Because we are working backwards, S(t,j) = (I + H(j)'*S(t,j+1)*(I + H(j))
  // which is what we want
  MatrixXd S_post;
  if (using_min_coords_) {
    MatrixXd P_pre =
        getMinimalCoordBasis(getReverseTime(t), getReverseMode(contact_mode));
    MatrixXd S_pre_max =
        P_pre.transpose() * S_pre * P_pre;  // H functions in the

    S_post = (I + H).transpose() * S_pre_max * (I + H);
    MatrixXd P_post = getMinimalCoordBasis(getReverseTime(t),
        getReverseMode(contact_mode + 1));
    S_post = P_post * S_post * P_post.transpose();
  } else {
    S_post = (I + H).transpose() * S_pre * (I + H);
  }
  return S_post;
}

MatrixXd HybridLQRController::calcJumpMapNaive(const MatrixXd& S_pre,
                                               int contact_mode, double t) {
  MatrixXd R(n_x_, n_x_);
  MatrixXd placeholder(n_x_, n_x_);
  calcLinearResetMap(t, contact_mode, &R, &placeholder);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);

  MatrixXd S_post;
  if (using_min_coords_) {
    MatrixXd P =
        getMinimalCoordBasis(getReverseTime(t), getReverseMode(contact_mode));
    MatrixXd S_pre_max = P.transpose() * S_pre * P;
    S_post = (I + R).transpose() * S_pre_max * (I + R);
    P = getMinimalCoordBasis(getReverseTime(t),
        getReverseMode(contact_mode + 1));
    S_post = P * S_post * P.transpose();
  } else {
    S_post = (I + R).transpose() * S_pre * (I + R);
  }
  // This only holds if we work backwards
  return S_post;
}

void HybridLQRController::calcLinearResetMap(double t, int contact_mode,
                                             MatrixXd* R, MatrixXd* Dx_R) {
  DRAKE_ASSERT(R->cols() == R->rows());
  DRAKE_ASSERT(R->cols() == n_x_);
  double t_rev = getReverseTime(t);
  int rev_mode = getReverseMode(contact_mode);
  VectorXd state = state_trajs_[rev_mode]->value(t_rev);
  VectorXd input = input_trajs_[rev_mode]->value(t_rev);

  VectorXd xu(n_x_ + n_u_);
  xu << state, input;

  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);
  std::unique_ptr<drake::systems::Context<AutoDiffXd>> context =
      createContext(plant_ad_, x_autodiff, u_autodiff);
  const drake::multibody::Frame<AutoDiffXd>& contact_frame =
      *contact_info_ad_[contact_mode].frameA[0];
  MatrixX<AutoDiffXd> J3d(3, n_v_);
  AutoDiffVecXd pt_cast = VectorXd::Zero(3);  // no offset from foot position
  const drake::multibody::Frame<AutoDiffXd>& world = plant_ad_.world_frame();
  this->plant_ad_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, contact_frame,
      pt_cast, world, world, &J3d);

  MatrixX<AutoDiffXd> J = TXZ_ * J3d;

  MatrixX<AutoDiffXd> M(n_v_, n_v_);
  plant_ad_.CalcMassMatrixViaInverseDynamics(*context, &M);
  MatrixX<AutoDiffXd> M_inv = M.inverse();
  MatrixX<AutoDiffXd> R_non_linear =
      -M_inv * J.transpose() * (J * M_inv * J.transpose()).inverse() * J;
  // The reset map for qdot
  *R = MatrixXd::Zero(n_x_, n_x_);
  *Dx_R = MatrixXd::Zero(n_x_, n_x_);

  MatrixX<AutoDiffXd> delta = R_non_linear * x_autodiff.tail(n_v_);
  MatrixXd R_linear = autoDiffToGradientMatrix(delta).block(0, 0, n_v_, n_x_);
  //  R->block(n_q_, n_q_, n_v_, n_v_) = autoDiffToValueMatrix(R_non_linear);
  R->block(n_q_, 0, n_v_, n_x_) = R_linear;
  Dx_R->block(n_q_, 0, n_v_, n_x_) = R_linear;
}

VectorXd HybridLQRController::calcLdot(double t, const VectorXd& l,
                                       const VectorXd&) {
  MatrixXd A = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);

  calcLinearizedDynamics(t, getContactModeAtTime(t), &A, &B);

  VectorXd l_copy(l);
  MatrixXd L = Map<MatrixXd>(l_copy.data(), sqrt(l.size()), sqrt(l.size()));
  MatrixXd lDot;
  if (using_min_coords_) {
    MatrixXd P =
        getMinimalCoordBasis(getReverseTime(t), 2 - getContactModeAtTime(t));
    MatrixXd A_bar = P * A * P.transpose();
    MatrixXd B_bar = P * B;
    lDot =
        -0.5 * P * Q_ * P.transpose() * L.transpose().inverse() -
            A_bar.transpose() * L +
            0.5 * L * L.transpose() * B_bar * R_.inverse() * B_bar.transpose()
                * L;
  } else {
    lDot = -0.5 * Q_ * L.transpose().inverse() - A.transpose() * L +
        0.5 * L * L.transpose() * B * R_.inverse() * B.transpose() * L;
  }
  lDot = -1 * lDot;

  return Map<VectorXd>(lDot.data(), l.size());
}

double HybridLQRController::getReverseTime(double t) const {
  if (t > state_trajs_[2]->end_time()) {
    //    return state_trajs_[2].end_time();
    return 0;
  }
  return state_trajs_[2]->end_time() - t;
}

int HybridLQRController::getContactModeAtTime(double t) const {
  for (int mode = 0; mode < num_modes_; ++mode) {
    if (t < impact_times_rev_[2 * mode + 1]) {
      return mode;
    }
  }
  return num_modes_ - 1;  // final contact mode
}

MatrixXd HybridLQRController::getSAtTimestamp(double t, int fsm_state) const {
  double t_rev = getReverseTime(t);
  int contact_mode_rev = getReverseMode(fsm_state);
  VectorXd l;
  if (t_rev < l_traj_[contact_mode_rev].start_time()) {
    l = l_traj_[contact_mode_rev].value(l_traj_[contact_mode_rev].start_time());
  } else if (t_rev > l_traj_[contact_mode_rev].end_time()) {
    l = l_traj_[contact_mode_rev].value(l_traj_[contact_mode_rev].end_time());
  } else {
    l = l_traj_[contact_mode_rev].value(t_rev);
  }
  MatrixXd L = Map<MatrixXd>(l.data(), sqrt(l.size()), sqrt(l.size()));
  return L * L.transpose();
}

bool duringImpact(const drake::lcmt_contact_results_for_viz& contact_info) {
  double threshold = 400;
  if (contact_info.num_point_pair_contacts == 2) {
    return (abs(contact_info.point_pair_contact_info[0].contact_force[2]) >
        threshold ||
        abs(contact_info.point_pair_contact_info[1].contact_force[2]) >
            threshold);
  } else if (contact_info.num_point_pair_contacts == 1) {
    return abs(contact_info.point_pair_contact_info[0].contact_force[2]) >
        threshold;
  } else {
    return false;
  }
}

void HybridLQRController::CalcControl(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* output) const {
  auto* current_state =
      (OutputVector<double>*) this->EvalVectorInput(context, state_port_);
  const BasicVector<double>* fsm_state =
      (BasicVector<double>*) this->EvalVectorInput(context, fsm_port_);
  auto* contact_info = this->EvalAbstractInput(context, contact_port_);
  const auto& contact_info_msg =
      contact_info->get_value<drake::lcmt_contact_results_for_viz>();

  double timestamp = current_state->get_timestamp();
  auto current_time = static_cast<double>(timestamp);
  int mode = (int) fsm_state->get_value()(0);
  VectorXd u_sol(n_u_);
  if (current_time < 1e-7) {
    u_sol = VectorXd::Zero(n_u_);
  }
    //  else if (duringImpact(contact_info_msg)) {
    //    std::cout << "During impact: " << std::endl;
    //    u_sol = VectorXd::Zero(n_u_);
    //  }
  else {
    VectorXd x_error =
        current_state->GetState() - state_trajs_[mode]->value(current_time);
    MatrixXd M(n_v_, n_v_);
    std::unique_ptr<drake::systems::Context<double>> state_context =
        createContext(plant_, current_state->GetState(),
            current_state->GetEfforts());
    plant_.CalcMassMatrixViaInverseDynamics(*state_context, &M);
    MatrixXd B = plant_.MakeActuationMatrix();
    MatrixXd B_linear(n_x_, n_u_);
    B_linear << MatrixXd::Zero(n_q_, n_u_), M.inverse() * B;

    // get the corresponding s_traj
    MatrixXd S = getSAtTimestamp(current_time, mode);
    if (using_min_coords_) {
      MatrixXd P = getMinimalCoordBasis(current_time, mode);
      MatrixXd K_minimal = R_.inverse() * B_linear.transpose() * (P
          .transpose() * S * P);
      u_sol = -K_minimal * x_error;
    } else {
      MatrixXd K = -R_.inverse() * B_linear.transpose() * S;
      u_sol = K * (x_error);
    }
    u_sol = u_sol + input_trajs_[mode]->value(current_time); // feedforward term
  }

  for (int i = 0; i < u_sol.size(); ++i) {  // cap the actuator inputs
    u_sol(i) =
        std::min<double>(u_max_(i), std::max<double>(u_min_(i), u_sol(i)));
  }
  output->SetDataVector(u_sol);
  output->set_timestamp(current_state->get_timestamp());
}

MatrixXd generate_state_input_matrix(drake::systems::DenseOutput<double>& p_t,
                                     VectorXd& times) {
  int num_states = p_t.Evaluate(times[0]).size();
  MatrixXd states_matrix = MatrixXd::Zero(num_states, times.size());
  for (int i = 0; i < times.size(); ++i) {
    states_matrix.col(i) = p_t.Evaluate((times[i]));
  }

  return states_matrix;
}

// Forwards time
MatrixXd HybridLQRController::getMinimalCoordBasis(double t, int mode) const {
  VectorXd p = p_traj_[mode].value(t);
  return Map<MatrixXd>(p.data(), n_d_, n_x_);
}

void HybridLQRController::calcMinimalCoordBasis() {
  DRAKE_DEMAND(!impact_times_.empty());
  std::vector<LcmTrajectory::Trajectory> trajectories;
  std::vector<std::string> trajectory_names;

  for (int mode = 0; mode < num_modes_; ++mode) {
    double t0 = impact_times_[2 * mode];
    double tf = impact_times_[2 * mode + 1];
    VectorXd state = state_trajs_[mode]->value(t0);
    VectorXd input = input_trajs_[mode]->value(t0);

    int mode_rev = getReverseMode(mode);
    VectorXd xu(n_x_ + n_u_);
    xu << state, input;

    AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
    AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);
    AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);
    std::unique_ptr<drake::systems::Context<AutoDiffXd>> context =
        createContext(plant_ad_, x_autodiff, u_autodiff);
    const drake::multibody::Frame<AutoDiffXd>& contact_frame =
        *contact_info_ad_[mode_rev].frameA[0];
    MatrixX<AutoDiffXd> J3d(3, n_v_);
    AutoDiffVecXd pt_cast = VectorXd::Zero(3);  // no offset from foot position
    const drake::multibody::Frame<AutoDiffXd>& world = plant_ad_.world_frame();
    this->plant_ad_.CalcJacobianTranslationalVelocity(
        *context, drake::multibody::JacobianWrtVariable::kV, contact_frame,
        pt_cast, world, world, &J3d);

    MatrixX<AutoDiffXd> J = TXZ_ * J3d;
    MatrixXd J_q = autoDiffToValueMatrix(J);
    MatrixXd dJdq_dqdt(J_q.rows(), J_q.cols());
    MatrixXd dJdq_flat = autoDiffToGradientMatrix(J).leftCols(n_q_);
    for (int i = 0; i < J_q.rows(); ++i) {
      dJdq_dqdt.row(i) = dJdq_flat.block(J.cols() * i, 0, J.cols(), J.cols()) *
          state.tail(n_v_);
    }
    MatrixXd F =
        MatrixXd::Zero(J_q.rows() + J_q.rows(), J_q.cols() + J_q.cols());

    F.block(0, 0, J_q.rows(), J_q.cols()) = J_q;
    F.block(J_q.rows(), 0, J_q.rows(), J_q.cols()) = dJdq_dqdt;
    F.block(J_q.rows(), J_q.cols(), J_q.rows(), J_q.cols()) = J_q;
    HouseholderQR<MatrixXd> qr_decomp(F.transpose());
    MatrixXd q_decomp = qr_decomp.householderQ();

    MatrixXd P_0 = q_decomp.block(0, F.rows(), q_decomp.rows(),
        q_decomp.cols() - F.rows());
    P_0.transposeInPlace();
    VectorXd p0 = Map<VectorXd>(P_0.data(), (n_x_ - 2 * n_c_) * n_x_);
    VectorXd defaultParams(0);
    const InitialValueProblem<double>::OdeContext default_values(t0, p0,
        defaultParams);
    InitialValueProblem<double> ivp(
        [this](const double& t, const VectorXd& pdot,
               const VectorXd& k) -> VectorXd { return calcPdot(t, pdot, k); },
        default_values);
    cout << "Integrating from t0: " << t0 << " to tf: " << tf << endl;
    cout << "For contact mode: " << mode << endl;
    p_t_[mode] = ivp.DenseSolve(tf);  // store each segment

    LcmTrajectory::Trajectory traj_block;
    traj_block.traj_name = "P" + std::to_string(mode);
    traj_block.time_vector = VectorXd::LinSpaced(10000, t0, tf);
    traj_block.datapoints =
        generate_state_input_matrix(*p_t_[mode], traj_block.time_vector);
    traj_block.datatypes = vector<string>(n_d_ * n_x_);
    trajectories.push_back(traj_block);
    trajectory_names.push_back(traj_block.traj_name);
  }
  cout << "Saving P_traj" << endl;
  LcmTrajectory saved_traj(trajectories, trajectory_names, "P_traj",
      "Time varying minimal coordinates basis");
  saved_traj.writeToFile(folder_path_ + "P_traj");
  cout << "Saved P traj" << endl;
}

void HybridLQRController::calcCostToGo(const MatrixXd& S_f) {
  Eigen::LLT<MatrixXd> lltOfA(S_f);
  MatrixXd L_f = lltOfA.matrixL();

  std::vector<MatrixXd> s_jump(num_modes_ - 1);
  // REMEMBER, we are integrating "forwards" in backwards time
  double t0;
  double tf;
  for (int mode = 0; mode < num_modes_; ++mode) {
    t0 = impact_times_rev_[2 * mode];
    tf = impact_times_rev_[2 * mode + 1];

    VectorXd l_0 = Map<VectorXd>(L_f.data(), L_f.size());
    VectorXd defaultParams(0);
    const InitialValueProblem<double>::OdeContext default_values(t0, l_0,
        defaultParams);
    InitialValueProblem<double> ivp(
        [this](const double& t, const VectorXd& ldot,
               const VectorXd& k) -> VectorXd { return calcLdot(t, ldot, k); },
        default_values);
    cout << "Integrating from t0: " << t0 << " to tf: " << tf << endl;
    cout << "For contact mode: " << mode << endl;
    l_t_[mode] = ivp.DenseSolve(tf);  // store each segment

    if (mode < (num_modes_ - 1)) {
      cout << "Calculating jump map" << endl;
      VectorXd l_pre = l_t_[mode]->Evaluate(tf);
      MatrixXd L_pre =
          Map<MatrixXd>(l_pre.data(), sqrt(l_pre.size()), sqrt(l_pre.size()));
      MatrixXd S_pre = L_pre * L_pre.transpose();  // Retrieve S
      if (naive_approach_) {
        s_jump[mode] = calcJumpMapNaive(S_pre, mode, tf);
      } else {
        s_jump[mode] = calcJumpMap(S_pre, mode, tf);
      }
      //      std::cout << "S_pre map\n" << S_pre << std::endl;
      //      std::cout << "S_post map\n" << s_jump[mode] << std::endl;
      Eigen::LLT<MatrixXd> lltOfS(s_jump[mode]);  // reconstruct L_f
      L_f = lltOfS.matrixL();
    }
  }

  std::vector<LcmTrajectory::Trajectory> l_trajectories;
  std::vector<std::string> trajectory_names;

  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory traj_block;
    traj_block.traj_name = "L" + std::to_string(getReverseMode(mode));
    traj_block.time_vector = VectorXd::LinSpaced(1000, l_t_[mode]->start_time(),
        l_t_[mode]->end_time());
    traj_block.datapoints =
        generate_state_input_matrix(*l_t_[mode], traj_block.time_vector);
    if (using_min_coords_) {
      traj_block.datatypes = vector<string>(n_d_ * n_d_);
    } else
      traj_block.datatypes = vector<string>(n_x_ * n_x_);
    l_trajectories.push_back(traj_block);
    trajectory_names.push_back(traj_block.traj_name);
  }
  LcmTrajectory saved_traj(
      l_trajectories, trajectory_names, "L_traj",
      "Square root of the time varying cost to go");
  saved_traj.writeToFile(l_traj_filepath_);
}

int HybridLQRController::getReverseMode(int mode) const {
  return num_modes_ - 1 - mode;
}

VectorXd HybridLQRController::calcPdot(double t, const Eigen::VectorXd& p,
                                       const Eigen::VectorXd&) {
  // This is all in forward time
  VectorXd p_copy(p);
  MatrixXd P = Map<MatrixXd>(p_copy.data(), n_x_ - 2 * n_c_, n_x_);

  int mode = getContactModeAtTime(t);
  int mode_rev = 2 - mode;

  VectorXd state = state_trajs_[mode]->value(t);
  VectorXd input = input_trajs_[mode]->value(t);

  VectorXd xu(n_x_ + n_u_);
  xu << state, input;

  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);
  AutoDiffVecXd v_autodiff = xu_autodiff.segment(n_q_, n_v_);
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);
  std::unique_ptr<drake::systems::Context<AutoDiffXd>> context =
      createContext(plant_ad_, x_autodiff, u_autodiff);
  const drake::multibody::Frame<AutoDiffXd>& contact_frame =
      *contact_info_ad_[mode_rev].frameA[0];
  MatrixX<AutoDiffXd> J3d(3, n_v_);
  AutoDiffVecXd pt_cast = VectorXd::Zero(3);  // no offset from foot position
  const drake::multibody::Frame<AutoDiffXd>& world = plant_ad_.world_frame();
  this->plant_ad_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, contact_frame,
      pt_cast, world, world, &J3d);

  MatrixX<AutoDiffXd> J = TXZ_ * J3d;
  MatrixX<AutoDiffXd> M(n_v_, n_v_);
  plant_ad_.CalcMassMatrixViaInverseDynamics(*context, &M);
  MatrixX<AutoDiffXd> B = plant_ad_.MakeActuationMatrix();
  AutoDiffVecXd C(n_v_);
  plant_ad_.CalcBiasTerm(*context, &C);
  AutoDiffVecXd g = plant_ad_.CalcGravityGeneralizedForces(*context);

  MatrixX<AutoDiffXd> J3d_times_v =
      plant_ad_
          .CalcBiasForJacobianSpatialVelocity(
              *context, drake::multibody::JacobianWrtVariable::kV,
              contact_frame, pt_cast, world, world)
          .tail(3);
  MatrixX<AutoDiffXd> JdotV = TXZ_ * J3d_times_v;
  MatrixX<AutoDiffXd> M_inv = M.inverse();
  AutoDiffVecXd rhs(n_v_ + n_c_);
  rhs << B * u_autodiff - C + g, -JdotV;
  MatrixX<AutoDiffXd> lhs(n_v_ + n_c_, n_v_ + n_c_);
  lhs << M, -J.transpose(), J, MatrixXd::Zero(n_c_, n_c_);
  AutoDiffVecXd qddot_lambda = lhs.inverse() * rhs;
  AutoDiffVecXd qddot = qddot_lambda.head(n_v_);

  AutoDiffVecXd xdot(n_x_);

  xdot << x_autodiff.tail(n_v_), qddot;
  MatrixXd AB = drake::math::autoDiffToGradientMatrix(xdot);

  // A lot of repeated code

  MatrixXd A = AB.leftCols(n_x_);
  MatrixXd J_q = autoDiffToValueMatrix(J);

  MatrixXd dJdq_dqdt(J_q.rows(), J_q.cols());
  MatrixXd dJdq_flat = autoDiffToGradientMatrix(J).leftCols(n_q_);

  VectorXd dJdq_vec = dJdq_flat * state.tail(n_v_);
  dJdq_dqdt = Map<MatrixXd>(dJdq_vec.data(), J_q.rows(), J_q.cols());

  MatrixXd F = MatrixXd::Zero(J_q.rows() + J_q.rows(), J_q.cols() + J_q.cols());

  F.block(0, 0, J_q.rows(), J_q.cols()) = J_q;                    // top left
  F.block(J_q.rows(), 0, J_q.rows(), J_q.cols()) = dJdq_dqdt;     // bottom left
  F.block(J_q.rows(), J_q.cols(), J_q.rows(), J_q.cols()) = J_q;  // bottom
  // right
  MatrixXd Fdot = -F * A;

  // Reuse indices
  int begin = 0;
  int nd = n_x_ - 2 * n_c_;
  int offset = nd * nd;
  int pdot_index = 0;
  int num_rows = nd * nd + nd * 2 * n_c_;
  MatrixXd alpha_PPT = 1e-8 * (MatrixXd::Identity(nd, nd) - P * P.transpose());
  MatrixXd alpha_PFT = -1e-8 * (P * F.transpose());

  VectorXd pdot_rhs(num_rows);
  pdot_rhs << VectorXd::Zero(nd * nd), VectorXd::Zero(nd * 2 * n_c_);
  MatrixXd pdot_lhs = MatrixXd::Zero(num_rows, nd * n_x_);

  // For P portion of ODE
  //    for (int i = 0; i < nd; ++i) {
  for (int i = 0; i < nd; ++i) {
    for (int j = 0; j < i + 1; ++j) {  // only creates the diagonal
      pdot_lhs.block(j + i * nd, i * n_x_, 1, n_x_) += P.row(j);
      pdot_lhs.block(j + i * nd, j * n_x_, 1, n_x_) += P.row(i);
      pdot_rhs(j + i * nd) = alpha_PPT(i, j);
    }
  }
  MatrixXd PFdot = P * Fdot.transpose();
  // For F portion of ODE
  for (int i = 0; i < 2 * n_c_; ++i) {
    begin = offset + i * nd;
    for (int j = 0; j < nd; ++j) {
      pdot_index = j * n_x_;
      pdot_lhs.block(offset + j + nd * i, pdot_index, 1, n_x_) = F.row(i);
    }
    pdot_rhs.segment(begin, nd) = -PFdot.col(i) + alpha_PFT.col(i);  // Fdot
  }
  //  VectorXd pdot = pdot_lhs.householderQr().solve(pdot_rhs);  // Solve for
  //  pdot
  VectorXd pdot = pdot_lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                          .solve(pdot_rhs);  // Solve for pdot (min norm version)
  MatrixXd Pdot(nd, n_x_);
  for (int i = 0; i < nd; ++i) {
    Pdot.row(i) = pdot.segment(i * n_x_, n_x_).transpose();
  }  // Reconstruct Pdot matrix because pdot is row major

  return Map<VectorXd>(Pdot.data(), nd * n_x_);
}

}  // namespace dairlib::systems
