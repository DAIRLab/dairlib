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
    const vector<shared_ptr<Trajectory<double>>>& state_traj,
    const vector<shared_ptr<Trajectory<double>>>& input_traj,
    const vector<double>& impact_times, bool naive_approach,
    bool using_min_coords, bool calcP, bool calcL)
    : plant_(plant),
      plant_ad_(plant_ad),
      contact_info_(contact_info),
      Q_(Q),
      R_(R),
      Qf_(Qf),
      state_trajs_(state_traj),
      input_trajs_(input_traj),
      impact_times_(impact_times),
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
  DRAKE_DEMAND(impact_times.size() == 2 * contact_info.size());

  TXZ_ << 1, 0, 0, 0, 0, 1;

  // Not as simple as just reversing the vector. Need to change the times as
  // well
  impact_times_rev_ = std::vector<double>(impact_times_.size());
  for (size_t i = 0; i < impact_times_.size(); ++i) {  // iterate in reverse
    impact_times_rev_[i] = getReverseTime(impact_times_[i]);
  }
  std::reverse(impact_times_rev_.begin(), impact_times_rev_.end());
  //  std::reverse_copy(std::begin(impact_times_), std::end(impact_times_),
  //                    std::begin(impact_times_rev_));
  contact_info_rev_ = std::vector<ContactInfo<double>>(contact_info.size());
  contact_info_ad_ = std::vector<ContactInfo<AutoDiffXd>>(contact_info.size());
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
  cost_output_port_ =
      this->DeclareVectorOutputPort(BasicVector<double>(4 + 5 * n_x_ + 2 * 3),
                                    &HybridLQRController::CalcCost)
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  contact_port_ = this->DeclareAbstractInputPort(
                          "lcmt_contact_info",
                          drake::Value<drake::lcmt_contact_results_for_viz>{})
                      .get_index();

  l_trajs_ = std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>>(
      contact_info.size());
  p_t_ = std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>>(
      contact_info.size());

  MatrixXd S_f;
  if (using_min_coords_) {
    if (!calcP) {
      const LcmTrajectory& P_traj =
          LcmTrajectory("../projects/hybrid_lqr/saved_trajs/P_traj");

      const LcmTrajectory::Trajectory& P_mode0 = P_traj.getTrajectory("P0");
      const LcmTrajectory::Trajectory& P_mode1 = P_traj.getTrajectory("P1");
      const LcmTrajectory::Trajectory& P_mode2 = P_traj.getTrajectory("P2");

      p_traj_.push_back(PiecewisePolynomial<double>::Pchip(P_mode0.time_vector,
                                                           P_mode0.datapoints));
      p_traj_.push_back(PiecewisePolynomial<double>::Pchip(P_mode1.time_vector,
                                                           P_mode1.datapoints));
      p_traj_.push_back(PiecewisePolynomial<double>::Pchip(P_mode2.time_vector,
                                                           P_mode2.datapoints));

    } else {
      cout << "Calculating minimal coord basis and saving\n";
      calcMinimalCoordBasis();
    }
    MatrixXd P = calcMinimalCoordBasis(0.0, 0);
    S_f = P * Qf_ * P.transpose();
  } else {
    S_f = Qf_;
  }

  Eigen::LLT<MatrixXd> lltOfA(S_f);
  MatrixXd L_f = lltOfA.matrixL();

  std::vector<MatrixXd> s_jump(contact_info.size() - 1);  // not sure yet if we
  // should store this
  // REMEMBER, we are integrating "forwards" in backwards time
  double t0;
  double tf;
  for (size_t mode = 0; mode < contact_info.size(); ++mode) {
    t0 = impact_times_rev_[2 * mode];
    tf = impact_times_rev_[2 * mode + 1];

    VectorXd l_0 = Map<VectorXd>(L_f.data(), L_f.size());
    VectorXd defaultParams(0);
    const InitialValueProblem<double>::SpecifiedValues default_values(
        t0, l_0, defaultParams);
    InitialValueProblem<double> ivp(
        [this](const double& t, const VectorXd& ldot,
               const VectorXd& k) -> VectorXd { return calcLdot(t, ldot, k); },
        default_values);
    cout << "Integrating from t0: " << t0 << " to tf: " << tf << endl;
    cout << "For contact mode: " << mode << endl;
    l_trajs_[mode] = ivp.DenseSolve(tf);  // store each segment

    if (mode < (contact_info.size() - 1)) {  // Only if another jump is
      cout << "Calculating jump map" << endl;
      VectorXd l_pre = l_trajs_[mode]->Evaluate(tf);
      MatrixXd L_pre =
          Map<MatrixXd>(l_pre.data(), sqrt(l_pre.size()), sqrt(l_pre.size()));
      MatrixXd S_pre = L_pre * L_pre.transpose();  // Retrieve S
      if (naive_approach) {
        s_jump[mode] = calcJumpMapNaive(S_pre, mode, tf);
      } else {
        s_jump[mode] = calcJumpMap(S_pre, mode, tf);
      }
      std::cout << "S_pre map" << S_pre << std::endl;
      std::cout << "S_after map" << s_jump[mode] << std::endl;
      Eigen::LLT<MatrixXd> lltOfS(s_jump[mode]);  // reconstruct L_f
      L_f = lltOfS.matrixL();
    }
  }

  if (calcL) {
    std::vector<LcmTrajectory::Trajectory> l_trajectories;
    std::vector<std::string> trajectory_names;

    //    VectorXd segment_times(2 * num_modes);
    //    segment_times << 0,
    //        state_trajs_[0].start_time(),
    //        state_traj.get_segment_times().at(FLAGS_knot_points),
    //        state_traj.get_segment_times().at(2 * FLAGS_knot_points - 1),
    //        state_traj.get_segment_times().at(2 * FLAGS_knot_points),
    //        state_traj.end_time();
    for (size_t mode = 0; mode < contact_info.size(); ++mode) {
      LcmTrajectory::Trajectory traj_block;
      traj_block.traj_name =
          "l_traj" + std::to_string(contact_info.size() - 1 - mode);
      //      traj_block.time_vector =
      //          VectorXd::LinSpaced(1000, state_trajs_[mode]->start_time(),
      //                              state_trajs_[mode]->end_time());
      //      traj_block.time_vector = VectorXd::LinSpaced(
      //          1000, impact_times_rev_[2 * mode], impact_times_rev_[2 * mode
      //          + 1]);
      traj_block.time_vector = VectorXd::LinSpaced(
          1000, l_trajs_[mode]->start_time(), l_trajs_[mode]->end_time());
      traj_block.datapoints =
          generate_state_input_matrix(*l_trajs_[mode], traj_block.time_vector);
      if (using_min_coords_) {
        traj_block.datatypes = vector<string>(n_d_ * n_d_);
      } else
        traj_block.datatypes = vector<string>(n_x_ * n_x_);
      l_trajectories.push_back(traj_block);
      trajectory_names.push_back(traj_block.traj_name);
    }
    if (using_min_coords_) {
      LcmTrajectory saved_traj(
          l_trajectories, trajectory_names, "L_traj",
          "Square root of the time varying cost to go with min_coords");
      saved_traj.writeToFile("../projects/hybrid_lqr/saved_trajs/L_traj_min");
    } else {
      LcmTrajectory saved_traj(l_trajectories, trajectory_names, "L_traj",
                               "Square root of the time varying cost to go");
      saved_traj.writeToFile("../projects/hybrid_lqr/saved_trajs/L_traj");
    }
  }
}

void HybridLQRController::calcLinearizedDynamics(double t, int contact_mode,
                                                 MatrixXd* A_linear,
                                                 MatrixXd* B_linear) {
  DRAKE_DEMAND(A_linear->cols() == A_linear->rows());
  DRAKE_DEMAND(A_linear->cols() == n_x_);
  double t_rev = getReverseTime(t);
  int rev_mode = num_modes_ - 1 - contact_mode;
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
  int rev_mode = 2 - contact_mode;
  VectorXd x_t = state_trajs_[rev_mode]->value(t_rev);
  VectorXd input = input_trajs_[rev_mode]->value(t_rev);
  std::unique_ptr<drake::systems::Context<double>> context =
      createContext(plant_, x_t, input);
  const drake::multibody::Frame<double>& world = plant_.world_frame();

  MatrixXd M(n_v_, n_v_);
  plant_.CalcMassMatrixViaInverseDynamics(*context, &M);

  MatrixXd B = plant_.MakeActuationMatrix();
  VectorXd C(n_v_);
  plant_.CalcBiasTerm(*context, &C);
  VectorXd g = plant_.CalcGravityGeneralizedForces(*context);

  const drake::multibody::Frame<double>& contact_frame =
      *contact_info_rev_[contact_mode].frameA[0];

  VectorXd pt_cast = VectorXd::Zero(3);
  MatrixXd J3d(3, n_v_);

  plant_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, contact_frame,
      pt_cast, world, world, &J3d);
  MatrixXd J3d_times_v =
      plant_
          .CalcBiasForJacobianSpatialVelocity(
              *context, drake::multibody::JacobianWrtVariable::kV,
              contact_frame, pt_cast, world, world)
          .tail(3);
  MatrixXd J_pre = TXZ_ * J3d;
  MatrixXd JdotV = TXZ_ * J3d_times_v;
  MatrixXd M_inv = M.inverse();
  VectorXd rhs(n_v_ + n_c_);
  rhs << B * input - C + g, -JdotV;
  MatrixXd lhs(n_v_ + n_c_, n_v_ + n_c_);
  lhs << M, -J_pre.transpose(), J_pre, MatrixXd::Zero(n_c_, n_c_);
  VectorXd qddot_lambda = lhs.inverse() * rhs;
  VectorXd qddot = qddot_lambda.head(n_v_);
  VectorXd xdot_pre(n_x_);
  xdot_pre << x_t.tail(n_v_), qddot;  // assuming v is qdot

  // Reinitializing contact toolkit for new contact mode
  // Also reinitializing context
  int new_contact_mode = contact_mode + 1;
  rev_mode = 2 - new_contact_mode;
  x_t = state_trajs_[rev_mode]->value(t_rev);
  input = input_trajs_[rev_mode]->value(t_rev);
  context = createContext(plant_, x_t, input);

  const drake::multibody::Frame<double>& new_contact_frame =
      *contact_info_rev_[new_contact_mode].frameA[0];
  plant_.CalcJacobianTranslationalVelocity(
      *context, drake::multibody::JacobianWrtVariable::kV, new_contact_frame,
      pt_cast, world, world, &J3d);
  J3d_times_v = plant_
                    .CalcBiasForJacobianSpatialVelocity(
                        *context, drake::multibody::JacobianWrtVariable::kV,
                        new_contact_frame, pt_cast, world, world)
                    .tail(3);
  MatrixXd J_post = TXZ_ * J3d;
  JdotV = TXZ_ * J3d_times_v;
  rhs << B * input - C + g, -JdotV;
  lhs << M, -J_post.transpose(), J_post, MatrixXd::Zero(n_c_, n_c_);
  qddot_lambda = lhs.inverse() * rhs;
  qddot = qddot_lambda.head(n_v_);
  VectorXd xdot_post(n_x_);
  xdot_post << x_t.tail(n_v_), qddot;  // assuming v is qdot

  // Now calculate H
  MatrixXd R = MatrixXd::Zero(n_x_, n_x_);
  calcLinearResetMap(t, contact_mode, &R);
  VectorXd dG(n_x_);

  dG << J_pre.row(1).transpose(), VectorXd::Zero(n_q_);  // dphi/dq, dphi/dqdot
  VectorXd J_delta = R * xdot_pre;
  double J_g = (J_pre.row(1) * xdot_pre.head(n_q_));

  MatrixXd H = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd I = MatrixXd::Identity(S_pre.rows(), S_pre.cols());

  cout << "xdot post: " << xdot_post << endl;
  cout << "xdot pre: " << xdot_pre << endl;
  cout << "J_delta: " << J_delta << endl;
  cout << "J: " << J_pre << endl;
  cout << "dG: " << dG << endl;
  std::cout << "Additional term: "
            << ((xdot_post - xdot_pre - J_delta) / J_g) * dG.transpose()
            << std::endl;
  std::cout << "R: " << R << std::endl;
  H << (R + ((xdot_post - xdot_pre - J_delta) / J_g) * dG.transpose());
  // Because we are working backwards, S(t,j) = (I + H(j)'*S(t,j+1)*(I + H(j))
  // which is what we want
  MatrixXd S_post;
  if (using_min_coords_) {
    MatrixXd P = calcMinimalCoordBasis(getReverseTime(t), 2 - contact_mode);
    //    MatrixXd H_bar = P * H * P.transpose();  // H functions in the same
    MatrixXd S_pre_max = P.transpose() * S_pre * P;  // H functions in the
    // same
    // state-space as R
    S_post = (I + H).transpose()
        * S_pre_max * (I + H);
    P = calcMinimalCoordBasis(getReverseTime(t), 2 - (contact_mode + 1));
    S_post = P * S_post * P.transpose();
  } else {
    S_post = (I + H).transpose() * S_pre * (I + H);
  }
  //  MatrixXd S_post(n_x_, n_x_);
  //  S_post = (I + H).transpose() * S_pre * (I + H);
  return S_post;
}

MatrixXd HybridLQRController::calcJumpMapNaive(const MatrixXd& S_pre,
                                               int contact_mode, double t) {
  MatrixXd R(n_x_, n_x_);
  calcLinearResetMap(t, contact_mode, &R);
  //  std::cout << "R: " << R << std::endl;
  MatrixXd I = MatrixXd::Identity(n_x_, n_x_);

  MatrixXd S_post;
  if (using_min_coords_) {
    MatrixXd P = calcMinimalCoordBasis(getReverseTime(t), 2 - contact_mode);
    MatrixXd S_pre_max = P.transpose() * S_pre * P;
    //    MatrixXd R_bar = P * R * P.transpose();
    //    S_post = (I + R_bar.transpose()) * S_pre * (I + R_bar);
    S_post = (I + R).transpose() * S_pre_max * (I + R);
    P = calcMinimalCoordBasis(getReverseTime(t), 2 - (contact_mode + 1));
    S_post = P * S_post * P.transpose();
  } else {
    S_post = (I + R).transpose() * S_pre * (I + R);
  }
  // This only holds if we work backwards
  return S_post;
}

void HybridLQRController::calcLinearResetMap(double t, int contact_mode,
                                             MatrixXd* R) {
  DRAKE_ASSERT(R->cols() == R->rows());
  DRAKE_ASSERT(R->cols() == n_x_);
  double t_rev = getReverseTime(t);
  int rev_mode = 2 - contact_mode;
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
  // The reset map for qdot
  MatrixX<AutoDiffXd> R_non_linear =
      -M_inv * J.transpose() * (J * M_inv * J.transpose()).inverse() * J;
  MatrixX<AutoDiffXd> delta = R_non_linear * x_autodiff.tail(n_v_);
  *R = MatrixXd::Zero(n_x_, n_x_);
  MatrixXd R_linear = autoDiffToGradientMatrix(delta).block(0, 0, n_v_, n_x_);
  R->block(0, 0, n_q_, n_q_) = MatrixXd::Identity(n_q_, n_q_);
  //  R->block(n_q_, n_q_, n_q_, n_q_) = autoDiffToValueMatrix(R_non_linear);
  R->block(n_q_, 0, n_v_, n_x_) = R_linear;
  *R = *R - MatrixXd::Identity(n_x_, n_x_);

  //  AutoDiffVecXd C(n_v_);
  //  plant_ad_.CalcBiasTerm(*context, &C);
  //  std::cout << "M: " << M << std::endl;
  //  std::cout << "J: " << J << std::endl;
  //  std::cout << "B: " << plant_.MakeActuationMatrix() << std::endl;
  //  std::cout << "Coriolis: " << autoDiffToValueMatrix(C) << std::endl;
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
        calcMinimalCoordBasis(getReverseTime(t), 2 - getContactModeAtTime(t));
    //    cout << "mode: " << 2 - getContactModeAtTime(t) << endl;
    //    cout << "P at time t: " << getReverseTime(t) << "\n" << P << endl;
    //    cout << "P PT : " << P * P.transpose() << endl;
    MatrixXd A_bar = P * A * P.transpose();
    MatrixXd B_bar = P * B;
    lDot =
        -0.5 * P * Q_ * P.transpose() * L.transpose().inverse() -
        A_bar.transpose() * L +
        0.5 * L * L.transpose() * B_bar * R_.inverse() * B_bar.transpose() * L;
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
  for (int mode = 0; mode < contact_info_rev_.size(); ++mode) {
    if (t < impact_times_rev_[2 * mode + 1]) {
      return mode;
    }
  }
  return contact_info_rev_.size() - 1;  // final contact mode
}

MatrixXd HybridLQRController::getSAtTimestamp(double t,
                                              double fsm_state) const {
  double t_rev = getReverseTime(t);
  size_t contact_mode_rev = contact_info_.size() - ((int)fsm_state + 1);
  //  size_t contact_mode_rev = fsm_state;
  VectorXd l;
  if (t_rev < l_trajs_[contact_mode_rev]->start_time()) {
    l = l_trajs_[contact_mode_rev]->Evaluate(
        l_trajs_[contact_mode_rev]->start_time());
  } else if (t_rev > l_trajs_[contact_mode_rev]->end_time()) {
    l = l_trajs_[contact_mode_rev]->Evaluate(
        l_trajs_[contact_mode_rev]->end_time());
  } else {
    l = l_trajs_[contact_mode_rev]->Evaluate(t_rev);
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
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const BasicVector<double>* fsm_state =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  auto* contact_info = this->EvalAbstractInput(context, contact_port_);
  const auto& contact_info_msg =
      contact_info->get_value<drake::lcmt_contact_results_for_viz>();

  double timestamp = current_state->get_timestamp();
  auto current_time = static_cast<double>(timestamp);
  int mode = (int)fsm_state->get_value()(0);
  VectorXd u_sol(n_u_);
  if (current_time < 1e-7) {
    u_sol = VectorXd::Zero(n_u_);
  } else if (duringImpact(contact_info_msg)) {
    std::cout << "During impact: " << std::endl;
    u_sol = VectorXd::Zero(n_u_);
  } else {
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
      MatrixXd P = calcMinimalCoordBasis(current_time, mode);
      MatrixXd K_minimal = -R_.inverse() * (P * B_linear).transpose() * S;
      u_sol = K_minimal * (P * x_error);
    } else {
      MatrixXd K = -R_.inverse() * B_linear.transpose() * S;
      u_sol = K * (x_error);
    }
  }

  for (int i = 0; i < u_sol.size(); ++i) {  // cap the actuator inputs
    u_sol(i) = std::min<double>(300, std::max<double>(-300, u_sol(i)));
  }
  output->SetDataVector(u_sol);
  output->set_timestamp(current_state->get_timestamp());
}  // namespace dairlib::systems

void HybridLQRController::CalcCost(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  auto* current_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  auto* contact_info = this->EvalAbstractInput(context, contact_port_);
  const BasicVector<double>* fsm_state =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  //  const BasicVector<double>* fsm_state =
  //      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  double timestamp = current_state->get_timestamp();
  auto current_time = static_cast<double>(timestamp);
  const auto& contact_info_msg =
      contact_info->get_value<drake::lcmt_contact_results_for_viz>();

  double cost0, cost1, cost2, cost3;
  if (current_time < 1e-7) {  // arbitrary small time window
    cost0 = 0;
    cost1 = 0;
    cost2 = 0;
    cost3 = 0;
  } else {
    //    cost0 = current_state->GetState()(5);            // left knee
    //    cost1 = state_trajs_.value(current_time)(5);      // left knee
    //    cost2 = current_state->GetState()(5 + 7);        // left knee
    //    cost3 = state_trajs_.value(current_time)(5 + 7);  // left knee
    int mode = (int)fsm_state->get_value()(0);
    MatrixXd S0 = getSAtTimestamp(current_time, 0);
    MatrixXd S1 = getSAtTimestamp(current_time, 1);
    MatrixXd S2 = getSAtTimestamp(current_time, 2);
    if (using_min_coords_) {
      VectorXd x_error0 =
          current_state->GetState() - state_trajs_[0]->value(current_time);
      VectorXd x_error1 =
          current_state->GetState() - state_trajs_[1]->value(current_time);
      VectorXd x_error2 =
          current_state->GetState() - state_trajs_[2]->value(current_time);
      MatrixXd P0 = calcMinimalCoordBasis(current_time, 0);
      MatrixXd P1 = calcMinimalCoordBasis(current_time, 1);
      MatrixXd P2 = calcMinimalCoordBasis(current_time, 2);
      cost0 = x_error0.transpose() * P0.transpose() * S0 * P0 * x_error0;
      cost1 = x_error1.transpose() * P1.transpose() * S1 * P1 * x_error1;
      cost2 = x_error2.transpose() * P2.transpose() * S2 * P2 * x_error2;
      //      Map<VectorXd> VectorXd(
      //          contact_info_msg.point_pair_contact_info[0].contact_force, 3);
      VectorXd grf(6);
      if (contact_info_msg.num_point_pair_contacts == 2) {
        if (contact_info_msg.point_pair_contact_info[0].body1_name ==
            "left_lower_leg(2)") {
          grf << Vector3d(
              contact_info_msg.point_pair_contact_info[0].contact_force),
              Vector3d(
                  contact_info_msg.point_pair_contact_info[1].contact_force);
        } else {
          grf << Vector3d(
              contact_info_msg.point_pair_contact_info[1].contact_force),
              Vector3d(
                  contact_info_msg.point_pair_contact_info[0].contact_force);
        }
      } else {
        if (contact_info_msg.point_pair_contact_info[0].body1_name ==
            "left_lower_leg(2)") {
          grf << Vector3d(
              contact_info_msg.point_pair_contact_info[0].contact_force),
              0, 0, 0;
        } else {
          grf << 0, 0, 0,
              Vector3d(
                  contact_info_msg.point_pair_contact_info[0].contact_force);
        }
      }
      output->get_mutable_value() << current_time, cost0, cost1, cost2,
          current_state->GetState(), state_trajs_[0]->value(current_time),
          state_trajs_[1]->value(current_time),
          state_trajs_[2]->value(current_time),
          state_trajs_[mode]->value(current_time), grf;
      return;  // TODO: clean up
    } else {
      VectorXd x_error =
          current_state->GetState() - state_trajs_[mode]->value(current_time);
      cost0 = x_error.transpose() * S0 * x_error;
      cost1 = x_error.transpose() * S1 * x_error;
      cost2 = x_error.transpose() * S2 * x_error;
      cost3 = 0;
    }
  }
  output->get_mutable_value() << current_time, cost0, cost1, cost2, cost3;
}  // namespace dairlib::systems

// drake::systems::EventStatus HybridLQRController::DiscreteVariableUpdate(
//    const drake::systems::Context<double>& context,
//    drake::systems::DiscreteValues<double>* discrete_state) const {
//  // Read in finite state machine
//  const BasicVector<double>* fsm_output =
//      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
//  VectorXd fsm_state = fsm_output->get_value();
//
//  auto prev_fsm_state =
//      discrete_state->get_mutable_vector(fsm_index_).get_mutable_value();
//
//  prev_fsm_state << fsm_state(0);
//  return EventStatus::Succeeded();
//}

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
MatrixXd HybridLQRController::calcMinimalCoordBasis(double t, int mode) const {
  VectorXd p = p_traj_[mode].value(t);
  return Map<MatrixXd>(p.data(), n_d_, n_x_);
}

void HybridLQRController::calcMinimalCoordBasis() {
  DRAKE_DEMAND(!impact_times_.empty());
  std::vector<LcmTrajectory::Trajectory> trajectories;
  std::vector<std::string> trajectory_names;

  for (size_t mode = 0; mode < contact_info_.size(); ++mode) {
    double t0 = impact_times_[2 * mode];
    double tf = impact_times_[2 * mode + 1];
    VectorXd state = state_trajs_[mode]->value(t0);
    VectorXd input = input_trajs_[mode]->value(t0);

    int mode_rev = contact_info_rev_.size() - 1 - mode;
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
    //    cout << "P0: " << P_0 << endl;
    VectorXd p0 = Map<VectorXd>(P_0.data(), (n_x_ - 2 * n_c_) * n_x_);
    VectorXd defaultParams(0);
    const InitialValueProblem<double>::SpecifiedValues default_values(
        t0, p0, defaultParams);
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
  LcmTrajectory saved_traj(trajectories, trajectory_names, "P_traj",
                           "Time varying minimal coordinates basis");
  saved_traj.writeToFile("../projects/hybrid_lqr/saved_trajs/P_traj");
  //  }
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
  //  pdot_rhs << Map<VectorXd>(alpha_PPT.data(), nd * nd),
  //      VectorXd::Zero(nd * 2 * n_c_);
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
