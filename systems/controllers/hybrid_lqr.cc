#include "hybrid_lqr.h"

#include <drake/lcmt_contact_results_for_viz.hpp>
#include "multibody/multibody_utils.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

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
using multibody::KinematicEvaluatorSet;
using multibody::WorldPointEvaluator;
using multibody::setContext;
using std::cout;
using std::endl;
using std::make_unique;
using std::shared_ptr;
using std::to_string;
using std::vector;

MatrixXd generate_state_input_matrix(drake::systems::DenseOutput<double>&,
                                     VectorXd& times);

HybridLQRController::HybridLQRController(
    const MultibodyPlant<double>& plant,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const vector<KinematicEvaluatorSet<AutoDiffXd>*>& contact_info,
    const vector<PiecewisePolynomial<double>>& state_trajs,
    const vector<PiecewisePolynomial<double>>& input_trajs, const MatrixXd& Q,
    const MatrixXd& R, const MatrixXd& Qf, double buffer_time,
    bool adjusted_reset_map, string folder_path, bool recalculateP,
    bool recalculateL)
    : plant_(plant),
      plant_ad_(plant_ad),
      contact_info_(contact_info),
      state_trajs_(state_trajs),
      input_trajs_(input_trajs),
      Q_(Q),
      R_(R),
      Qf_(Qf),
      buffer_time_(buffer_time),
      adjusted_reset_map_(adjusted_reset_map),
      folder_path_(folder_path),
      n_q_(plant.num_positions()),
      n_v_(plant.num_velocities()),
      n_x_(n_q_ + n_v_),
      n_u_(plant.num_actuators()),
      n_c_(2),
      n_d_(n_x_ - 2 * n_c_),
      num_modes_(state_trajs_.size()) {
  DRAKE_ASSERT(contact_info.size() == num_modes_)
  DRAKE_ASSERT(input_trajs.size() == num_modes_)

  context_ = plant_.CreateDefaultContext().get();

  // Declare all system ports
  // state feedback
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  // control efforts output port
  efforts_port_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(n_u_),
                                    &HybridLQRController::CalcControl)
          .get_index();
  // finite state machine
  input_port_fsm_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  // contact info (unused)
  input_port_contact_ =
      this->DeclareAbstractInputPort(
              "lcmt_contact_info",
              drake::Value<drake::lcmt_contact_results_for_viz>{})
          .get_index();

  TXZ_ << 1, 0, 0, 0, 0, 1;

  // Create the vector of impact times and the corresponding impact times in
  // reverse time
  impact_times_rev_ = std::vector<double>(impact_times_.size());
  for (int i = 0; i < num_modes_; ++i) {
    impact_times_.push_back(state_trajs[i].start_time());
    impact_times_rev_.push_back(getReverseTime(state_trajs[i].start_time()));
    impact_times_.push_back(state_trajs[i].end_time());
    impact_times_rev_.push_back(getReverseTime(state_trajs[i].end_time()));
  }
  std::reverse(impact_times_rev_.begin(), impact_times_rev_.end());

  // Set the actuator limits
  u_min_ = VectorXd::Zero(n_u_);
  u_max_ = VectorXd::Zero(n_u_);
  for (JointActuatorIndex i(0); i < n_u_; i++) {
    u_min_(i) = -plant.get_joint_actuator(i).effort_limit();
    u_max_(i) = plant.get_joint_actuator(i).effort_limit();
  }

  // Set the filepath for the different options
  l_traj_filepath_ = folder_path_ + "L_traj";
  if (!adjusted_reset_map_) l_traj_filepath_ = l_traj_filepath_ + "_adjusted";

  if (recalculateP) {
    cout << "Calculating and saving the minimal coord basis\n";
    calcMinimalCoordBasis();
  }
  /*
   * Load the P(t) trajectory
   */
  const LcmTrajectory& P_traj = LcmTrajectory(folder_path_ + "P_traj");
  for (int mode = 0; mode < num_modes_; ++mode) {
    const LcmTrajectory::Trajectory& P_mode_i =
        P_traj.GetTrajectory("P" + to_string(mode));
    p_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
        P_mode_i.time_vector, P_mode_i.datapoints));
  }

  if (recalculateL) {
    MatrixXd S_f;
    MatrixXd P = getPAtTime(0.0, 0);
    S_f = P * Qf_ * P.transpose();
    calcCostToGo(S_f);
  }

  /*
   * Load the L(t) trajectory
   */
  const LcmTrajectory& L_traj = LcmTrajectory(l_traj_filepath_);
  for (int mode = 0; mode < num_modes_; ++mode) {
    const LcmTrajectory::Trajectory& L_mode_i =
        L_traj.GetTrajectory("L" + to_string(mode));
    l_traj_.push_back(PiecewisePolynomial<double>::FirstOrderHold(
        L_mode_i.time_vector, L_mode_i.datapoints));
  }
}  // namespace dairlib::systems

void HybridLQRController::CalcControl(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* output) const {
  auto* current_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const BasicVector<double>* fsm_state =
      (BasicVector<double>*)this->EvalVectorInput(context, input_port_fsm_);

  double timestamp = current_state->get_timestamp();
  int mode = (int)fsm_state->get_value()(0);
  VectorXd u_sol(n_u_);
  //  if (timestamp < 1e-7) {
  //    u_sol = VectorXd::Zero(n_u_);
  //  } else
  // Zero out controller efforts in a window around the nominal time
  if (abs(timestamp - state_trajs_[0].end_time()) < 0.5 * buffer_time_ ||
      abs(timestamp - state_trajs_[1].end_time()) < 0.5 * buffer_time_) {
    u_sol = VectorXd::Zero(n_u_);
  } else {
    plant_.SetPositionsAndVelocities(context_, current_state->get_value());
    MatrixXd M(n_v_, n_v_);
    plant_.CalcMassMatrix(*context_, &M);
    MatrixXd B = plant_.MakeActuationMatrix();
    MatrixXd B_linear(n_x_, n_u_);
    B_linear << MatrixXd::Zero(n_q_, n_u_), M.inverse() * B;

    // get S(t) and P(t)
    MatrixXd S = getSAtTime(timestamp, mode);
    MatrixXd P = getPAtTime(timestamp, mode);
    MatrixXd K_minimal =
        R_.inverse() * B_linear.transpose() * (P.transpose() * S * P);
    VectorXd x_error =
        current_state->GetState() - state_trajs_[mode].value(timestamp);
    u_sol = -K_minimal * x_error + input_trajs_[mode].value(timestamp);
  }

  for (unsigned int i = 0; i < u_sol.size(); ++i) {  // apply actuator limits
    u_sol(i) =
        std::min<double>(u_max_(i), std::max<double>(u_min_(i), u_sol(i)));
  }
  output->SetDataVector(u_sol);
  output->set_timestamp(current_state->get_timestamp());
}

// P(t) is calculated in forwards time
MatrixXd HybridLQRController::getPAtTime(double t, int mode) const {
  DRAKE_ASSERT(!p_traj_.empty());
  VectorXd p = p_traj_[mode].value(t);
  return Map<MatrixXd>(p.data(), n_d_, n_x_);
}

// L(t) is calculated in reverse time
MatrixXd HybridLQRController::getSAtTime(double t, int contact_mode) const {
  DRAKE_ASSERT(!l_traj_.empty());
  double t_rev = getReverseTime(t);
  int contact_mode_rev = getReverseMode(contact_mode);
  VectorXd l = l_traj_[contact_mode_rev].value(t_rev);
  MatrixXd L = Map<MatrixXd>(l.data(), sqrt(l.size()), sqrt(l.size()));
  return L * L.transpose();
}

void HybridLQRController::calcMinimalCoordBasis() {
  DRAKE_DEMAND(!impact_times_.empty());
  std::vector<LcmTrajectory::Trajectory> p_trajs;
  std::vector<std::string> trajectory_names;
  auto p_t_ = std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>>(
      num_modes_);

  for (int mode = 0; mode < num_modes_; ++mode) {
    double t0 = impact_times_[2 * mode];
    double tf = impact_times_[2 * mode + 1];
    VectorXd state = state_trajs_[mode].value(t0);
    VectorXd input = input_trajs_[mode].value(t0);

    int mode_rev = getReverseMode(mode);
    VectorXd xu(n_x_ + n_u_);
    xu << state, input;

    AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
    AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);
    AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);
    plant_ad_.SetPositionsAndVelocities(context_ad_, x_autodiff);

    MatrixX<AutoDiffXd> J =
        TXZ_ * contact_info_[mode_rev]->EvalFullJacobian(*context_ad_);

    MatrixXd J_q = autoDiffToValueMatrix(J);
    MatrixXd dJdt(J_q.rows(), J_q.cols());
    MatrixXd dJdq_flat = autoDiffToGradientMatrix(J).leftCols(n_q_);
    for (unsigned int i = 0; i < J_q.rows(); ++i) {
      dJdt.row(i) = dJdq_flat.block(J.cols() * i, 0, J.cols(), J.cols()) *
                    state.tail(n_v_);
    }

    // F is the linearized constraint
    MatrixXd F =
        MatrixXd::Zero(J_q.rows() + J_q.rows(), J_q.cols() + J_q.cols());
    F << J_q, MatrixXd::Zero(J_q.rows(), J_q.cols()), dJdt, J_q;
    //    F.block(0, 0, J_q.rows(), J_q.cols()) = J_q;
    //    F.block(J_q.rows(), 0, J_q.rows(), J_q.cols()) = dJdq_dqdt;
    //    F.block(J_q.rows(), J_q.cols(), J_q.rows(), J_q.cols()) = J_q;

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

    LcmTrajectory::Trajectory p_traj_block;
    p_traj_block.traj_name = "P" + std::to_string(mode);
    p_traj_block.time_vector = VectorXd::LinSpaced(RESOLUTION, t0, tf);
    p_traj_block.datapoints =
        generate_state_input_matrix(*p_t_[mode], p_traj_block.time_vector);
    p_traj_block.datatypes = vector<string>(n_d_ * n_x_);
    p_trajs.push_back(p_traj_block);
    trajectory_names.push_back(p_traj_block.traj_name);
  }
  LcmTrajectory saved_traj(p_trajs, trajectory_names, "P_traj",
                           "Time varying minimal coordinates basis");
  saved_traj.WriteToFile(folder_path_ + "P_traj");
  cout << "Saved P traj" << endl;
}

void HybridLQRController::calcCostToGo(const MatrixXd& S_f) {
  Eigen::LLT<MatrixXd> lltOfA(S_f);
  MatrixXd L_f = lltOfA.matrixL();

  auto l_t_ = std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>>(
      num_modes_);
  // REMEMBER, we are integrating in backwards time
  double t0;
  double tf;
  for (int mode = 0; mode < num_modes_; ++mode) {
    t0 = impact_times_rev_[2 * mode];
    tf = impact_times_rev_[2 * mode + 1];

    VectorXd l_0 = Map<VectorXd>(L_f.data(), L_f.size());
    const InitialValueProblem<double>::OdeContext default_values(
        t0, l_0, VectorXd::Zero(0));
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
      MatrixXd S_post = L_pre * L_pre.transpose();  // Retrieve S
      MatrixXd S_pre;
      if (adjusted_reset_map_) {
        S_pre = calcJumpMap(S_post, mode, tf);
      } else {
        S_pre = calcAdjustedJumpMap(S_post, mode, tf);
      }
      Eigen::LLT<MatrixXd> lltOfS(S_pre);  // reconstruct L_f
      L_f = lltOfS.matrixL();
    }
  }

  // Convert DenseOutput to trajectories and save them
  std::vector<LcmTrajectory::Trajectory> l_trajectories;
  std::vector<std::string> trajectory_names;

  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory traj_block;
    traj_block.traj_name = "L" + std::to_string(getReverseMode(mode));
    traj_block.time_vector = VectorXd::LinSpaced(
        RESOLUTION, l_t_[mode]->start_time(), l_t_[mode]->end_time());
    traj_block.datapoints =
        generate_state_input_matrix(*l_t_[mode], traj_block.time_vector);
    traj_block.datatypes = vector<string>(n_d_ * n_d_);
    l_trajectories.push_back(traj_block);
    trajectory_names.push_back(traj_block.traj_name);
  }
  LcmTrajectory saved_traj(l_trajectories, trajectory_names, "L_traj",
                           "Square root of the time varying cost to go");
  saved_traj.WriteToFile(l_traj_filepath_);
}

void HybridLQRController::calcLinearizedDynamics(double t, int contact_mode,
                                                 MatrixXd* A_linear,
                                                 MatrixXd* B_linear) {
  DRAKE_DEMAND(A_linear->cols() == A_linear->rows());
  DRAKE_DEMAND(A_linear->cols() == n_x_);
  double t_rev = getReverseTime(t);
  int rev_mode = getReverseMode(contact_mode);
  VectorXd state = state_trajs_[rev_mode].value(t_rev);
  VectorXd input = input_trajs_[rev_mode].value(t_rev);

  VectorXd xu(n_x_ + n_u_);
  xu << state, input;
  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);  // first segment
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);  // middle segment

  plant_ad_.SetPositionsAndVelocities(context_ad_, x_autodiff);

  const AutoDiffVecXd& xdot =
      contact_info_[rev_mode]->EvalFullTimeDerivative(*context_ad_);

  MatrixXd AB = autoDiffToGradientMatrix(xdot);

  *A_linear << AB.leftCols(n_x_);
  *B_linear << AB.block(0, n_x_, AB.rows(), n_u_);
}

MatrixXd HybridLQRController::calcJumpMap(const MatrixXd& S_post,
                                          int contact_mode, double t) {
  MatrixXd R(n_x_, n_x_);
  calcLinearResetMap(t, contact_mode, &R);
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);

  MatrixXd P = getPAtTime(getReverseTime(t), getReverseMode(contact_mode));
  // Map to maximum coordinates
  MatrixXd S_post_max = P.transpose() * S_post * P;
  // Compute jump map
  MatrixXd S_pre = (I + R).transpose() * S_post_max * (I + R);
  P = getPAtTime(getReverseTime(t), getReverseMode(contact_mode + 1));
  // Map back to minimum coordinates in new contact mode
  S_pre = P * S_pre * P.transpose();
  return S_pre;
}

MatrixXd HybridLQRController::calcAdjustedJumpMap(MatrixXd& S_post,
                                                  int contact_mode, double t) {
  double t_rev = getReverseTime(t);
  int rev_mode = getReverseMode(contact_mode + 1);
  //  int rev_mode = getReverseMode(contact_mode);
  VectorXd x_pre = state_trajs_[rev_mode].value(t_rev);
  VectorXd u_pre = input_trajs_[rev_mode].value(t_rev);

  VectorXd xu(n_x_ + n_u_);
  xu << x_pre, u_pre;
  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);  // first segment
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);  // middle segment

  plant_ad_.SetPositionsAndVelocities(context_ad_, x_autodiff);

  const MatrixX<AutoDiffXd>& J_pre =
      contact_info_[rev_mode]->EvalActiveJacobian(*context_ad_);
  MatrixXd J = TXZ_ * autoDiffToValueMatrix(J_pre);
  VectorXd xdot_pre = state_trajs_[rev_mode].derivative(1).value(
      state_trajs_[rev_mode].end_time());

  // Get xdot for the post-impact mode
  int rev_mode_post = getReverseMode(contact_mode);
  VectorXd xdot_post = state_trajs_[rev_mode_post].derivative(1).value(
      state_trajs_[rev_mode_post].end_time());

  // Now calculate H
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
  MatrixXd R = MatrixXd::Zero(n_x_, n_x_);
  calcLinearResetMap(t, contact_mode, &R);
  VectorXd D1_g(n_x_);

  D1_g << J.row(1).transpose(),
      VectorXd::Zero(n_q_);  // dphi/dq, dphi/dqdot
  VectorXd J_delta = R * xdot_pre;
  double J_g = (J.row(1) * xdot_pre.head(n_q_));

  MatrixXd H = MatrixXd::Zero(n_x_, n_x_);
  H << (R + ((xdot_post - xdot_pre - J_delta) / J_g) * D1_g.transpose());

  // Map to full coordinates in post-impact basis
  MatrixXd P_post = getPAtTime(getReverseTime(t), getReverseMode(contact_mode));
  MatrixXd S_post_max = P_post.transpose() * S_post * P_post;

  MatrixXd S_pre = (I + H).transpose() * S_post_max * (I + H);
  MatrixXd P_pre =
      getPAtTime(getReverseTime(t), getReverseMode(contact_mode + 1));
  S_pre = P_pre * S_pre * P_pre.transpose();
  return S_pre;
}

void HybridLQRController::calcLinearResetMap(double t, int contact_mode,
                                             Eigen::MatrixXd* R) {
  DRAKE_ASSERT(R->cols() == R->rows());
  DRAKE_ASSERT(R->cols() == n_x_);
  //  double t_rev = getReverseTime(t);
  int rev_mode = getReverseMode(contact_mode);
  double t_impact = state_trajs_[rev_mode].start_time();
  VectorXd state = state_trajs_[rev_mode].value(t_impact);
  VectorXd input = input_trajs_[rev_mode].value(t_impact);

  VectorXd xu(n_x_ + n_u_);
  xu << state, input;

  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);
  setContext<AutoDiffXd>(plant_ad_, x_autodiff, u_autodiff, context_ad_);

  MatrixX<AutoDiffXd> J =
      TXZ_ * contact_info_[rev_mode]->EvalActiveJacobian(*context_ad_);

  MatrixX<AutoDiffXd> M(n_v_, n_v_);
  plant_ad_.CalcMassMatrix(*context_ad_, &M);
  MatrixX<AutoDiffXd> M_inv = M.inverse();
  MatrixX<AutoDiffXd> R_non_linear =
      -M_inv * J.transpose() * (J * M_inv * J.transpose()).inverse() * J;

  // The reset map for qdot
  *R = MatrixXd::Zero(n_x_, n_x_);
  MatrixX<AutoDiffXd> delta = R_non_linear * x_autodiff.tail(n_v_);
  MatrixXd R_linear = autoDiffToGradientMatrix(delta).block(0, 0, n_v_, n_x_);
  R->block(n_q_, 0, n_v_, n_x_) = R_linear;
}

VectorXd HybridLQRController::calcLdot(double t, const VectorXd& l,
                                       const VectorXd&) {
  MatrixXd A = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);

  calcLinearizedDynamics(t, getContactModeAtTime(t), &A, &B);

  VectorXd l_copy(l);
  MatrixXd L = Map<MatrixXd>(l_copy.data(), sqrt(l.size()), sqrt(l.size()));
  MatrixXd lDot;
  MatrixXd P = getPAtTime(getReverseTime(t), 2 - getContactModeAtTime(t));
  MatrixXd A_bar = P * A * P.transpose();
  MatrixXd B_bar = P * B;
  lDot = -0.5 * P * Q_ * P.transpose() * L.transpose().inverse() -
         A_bar.transpose() * L +
         0.5 * L * L.transpose() * B_bar * R_.inverse() * B_bar.transpose() * L;
  lDot = -1 * lDot;

  return Map<VectorXd>(lDot.data(), l.size());
}

VectorXd HybridLQRController::calcPdot(double t, const Eigen::VectorXd& p,
                                       const Eigen::VectorXd&) {
  // This is all in forward time
  VectorXd p_copy(p);
  MatrixXd P = Map<MatrixXd>(p_copy.data(), n_x_ - 2 * n_c_, n_x_);

  int mode = getContactModeAtTime(t);
  int mode_rev = 2 - mode;

  VectorXd state = state_trajs_[mode].value(t);
  VectorXd input = input_trajs_[mode].value(t);

  VectorXd xu(n_x_ + n_u_);
  xu << state, input;

  AutoDiffVecXd xu_autodiff = initializeAutoDiff(xu);
  AutoDiffVecXd x_autodiff = xu_autodiff.head(n_x_);
  AutoDiffVecXd u_autodiff = xu_autodiff.tail(n_u_);
  setContext<AutoDiffXd>(plant_ad_, x_autodiff, u_autodiff, context_ad_);

  MatrixXd A = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd B = MatrixXd::Zero(n_x_, n_u_);

  calcLinearizedDynamics(t, getContactModeAtTime(t), &A, &B);
  MatrixX<AutoDiffXd> J = contact_info_[mode_rev]->EvalFullJacobian(*context_ad_);

  // A lot of repeated code
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
  int offset = n_d_ * n_d_;
  int pdot_index = 0;
  int num_rows = n_d_ * n_d_ + n_d_ * 2 * n_c_;
  MatrixXd alpha_PPT =
      ALPHA * (MatrixXd::Identity(n_d_, n_d_) - P * P.transpose());
  MatrixXd alpha_PFT = -ALPHA * (P * F.transpose());

  VectorXd pdot_rhs(num_rows);
  pdot_rhs << VectorXd::Zero(n_d_ * n_d_), VectorXd::Zero(n_d_ * 2 * n_c_);
  MatrixXd pdot_lhs = MatrixXd::Zero(num_rows, n_d_ * n_x_);

  // For P portion of ODE
  for (int i = 0; i < n_d_; ++i) {
    for (int j = 0; j < i + 1; ++j) {  // only creates the diagonal
      pdot_lhs.block(j + i * n_d_, i * n_x_, 1, n_x_) += P.row(j);
      pdot_lhs.block(j + i * n_d_, j * n_x_, 1, n_x_) += P.row(i);
      pdot_rhs(j + i * n_d_) = alpha_PPT(i, j);
    }
  }
  MatrixXd PFdot = P * Fdot.transpose();
  // For F portion of ODE
  for (int i = 0; i < 2 * n_c_; ++i) {
    begin = offset + i * n_d_;
    for (int j = 0; j < n_d_; ++j) {
      pdot_index = j * n_x_;
      pdot_lhs.block(offset + j + n_d_ * i, pdot_index, 1, n_x_) = F.row(i);
    }
    pdot_rhs.segment(begin, n_d_) = -PFdot.col(i) + alpha_PFT.col(i);  // Fdot
  }
  VectorXd pdot = pdot_lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                      .solve(pdot_rhs);  // Solve for pdot (min norm version)
  MatrixXd Pdot(n_d_, n_x_);
  for (int i = 0; i < n_d_; ++i) {
    Pdot.row(i) = pdot.segment(i * n_x_, n_x_).transpose();
  }  // Reconstruct Pdot matrix because pdot is row major

  return Map<VectorXd>(Pdot.data(), n_d_ * n_x_);
}

MatrixXd generate_state_input_matrix(drake::systems::DenseOutput<double>& p_t,
                                     VectorXd& times) {
  int num_states = p_t.Evaluate(times[0]).size();
  MatrixXd states_matrix = MatrixXd::Zero(num_states, times.size());
  for (unsigned int i = 0; i < times.size(); ++i) {
    states_matrix.col(i) = p_t.Evaluate((times[i]));
  }

  return states_matrix;
}

int HybridLQRController::getContactModeAtTime(double t) const {
  for (int mode = 0; mode < num_modes_; ++mode) {
    if (t < impact_times_rev_[2 * mode + 1]) {
      return mode;
    }
  }
  return num_modes_ - 1;  // final contact mode
}

double HybridLQRController::getReverseTime(double t) const {
  if (t > state_trajs_[2].end_time()) {
    return 0;
  }
  return state_trajs_[2].end_time() - t;
}

int HybridLQRController::getReverseMode(int mode) const {
  return num_modes_ - 1 - mode;
}

}  // namespace dairlib::systems
