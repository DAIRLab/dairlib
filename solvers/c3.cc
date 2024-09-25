#include "solvers/c3.h"

#include <chrono>
#include <iostream>

#include <Eigen/Core>
#include <omp.h>

#include "solvers/lcs.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::solvers::SolverOptions;

using drake::solvers::OsqpSolver;
using drake::solvers::OsqpSolverDetails;
using drake::solvers::Solve;

C3::CostMatrices::CostMatrices(const std::vector<Eigen::MatrixXd>& Q,
                               const std::vector<Eigen::MatrixXd>& R,
                               const std::vector<Eigen::MatrixXd>& G,
                               const std::vector<Eigen::MatrixXd>& U) {
  this->Q = Q;
  this->R = R;
  this->G = G;
  this->U = U;
}

C3::C3(const LCS& LCS, const C3::CostMatrices& costs,
       const vector<VectorXd>& x_desired, const C3Options& options)
    : warm_start_(options.warm_start),
      lcs_(LCS),
      N_((LCS.A_).size()),
      n_((LCS.A_)[0].cols()),
      m_((LCS.D_)[0].cols()),
      k_((LCS.B_)[0].cols()),
      A_(LCS.A_),
      B_(LCS.B_),
      D_(LCS.D_),
      d_(LCS.d_),
      E_(LCS.E_),
      F_(LCS.F_),
      H_(LCS.H_),
      c_(LCS.c_),
      Q_(costs.Q),
      R_(costs.R),
      U_(costs.U),
      G_(costs.G),
      x_desired_(x_desired),
      options_(options),
      h_is_zero_(H_[0].isZero(0)),
      prog_(MathematicalProgram()),
      osqp_(OsqpSolver()) {
  if (warm_start_) {
    warm_start_delta_.resize(options_.admm_iter + 1);
    warm_start_binary_.resize(options_.admm_iter + 1);
    warm_start_x_.resize(options_.admm_iter + 1);
    warm_start_lambda_.resize(options_.admm_iter + 1);
    warm_start_u_.resize(options_.admm_iter + 1);
    for (size_t iter = 0; iter < options_.admm_iter + 1; ++iter) {
      warm_start_delta_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_delta_[iter][i] = VectorXd::Zero(n_ + m_ + k_);
      }
      warm_start_binary_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_binary_[iter][i] = VectorXd::Zero(m_);
      }
      warm_start_x_[iter].resize(N_ + 1);
      for (size_t i = 0; i < N_ + 1; i++) {
        warm_start_x_[iter][i] = VectorXd::Zero(n_);
      }
      warm_start_lambda_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_lambda_[iter][i] = VectorXd::Zero(m_);
      }
      warm_start_u_[iter].resize(N_);
      for (size_t i = 0; i < N_; i++) {
        warm_start_u_[iter][i] = VectorXd::Zero(k_);
      }
    }
  }

  auto Dn = D_.at(0).norm();
  auto An = A_.at(0).norm();
  AnDn_ = An / Dn;

  for (int i = 0 ; i < N_; ++i){
    D_.at(i) *= AnDn_;
    E_.at(i) /= AnDn_;
    c_.at(i) /= AnDn_;
    H_.at(i) /= AnDn_;
  }

  x_ = vector<drake::solvers::VectorXDecisionVariable>();
  u_ = vector<drake::solvers::VectorXDecisionVariable>();
  lambda_ = vector<drake::solvers::VectorXDecisionVariable>();

  z_sol_ = std::make_unique<std::vector<VectorXd>>();
  x_sol_ = std::make_unique<std::vector<VectorXd>>();
  lambda_sol_ = std::make_unique<std::vector<VectorXd>>();
  u_sol_ = std::make_unique<std::vector<VectorXd>>();
  w_sol_ = std::make_unique<std::vector<VectorXd>>();
  delta_sol_ = std::make_unique<std::vector<VectorXd>>();
  for (int i = 0; i < N_; ++i) {
    z_sol_->push_back(Eigen::VectorXd::Zero(n_ + m_ + k_));
    x_sol_->push_back(Eigen::VectorXd::Zero(n_));
    lambda_sol_->push_back(Eigen::VectorXd::Zero(m_));
    u_sol_->push_back(Eigen::VectorXd::Zero(k_));
    w_sol_->push_back(Eigen::VectorXd::Zero(n_ + m_ + k_));
    delta_sol_->push_back(Eigen::VectorXd::Zero(n_ + m_ + k_));
  }

  for (int i = 0; i < N_ + 1; i++) {
    x_.push_back(prog_.NewContinuousVariables(n_, "x" + std::to_string(i)));
    if (i < N_) {
      u_.push_back(prog_.NewContinuousVariables(k_, "k" + std::to_string(i)));
      lambda_.push_back(
          prog_.NewContinuousVariables(m_, "lambda" + std::to_string(i)));
    }
  }

  MatrixXd LinEq(n_, 2 * n_ + m_ + k_);
  LinEq.block(0, n_ + m_ + k_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);
  dynamics_constraints_.resize(N_);
  target_cost_.resize(N_ + 1);
  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, m_) = D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = B_.at(i);

    //    prog_.AddLinearEqualityConstraint(
    //        LinEq, -d_.at(i), {x_.at(i), lambda_.at(i), u_.at(i), x_.at(i +
    //        1)});
    dynamics_constraints_[i] =
        prog_
            .AddLinearEqualityConstraint(
                LinEq, -d_.at(i),
                {x_.at(i), lambda_.at(i), u_.at(i), x_.at(i + 1)})
            .evaluator()
            .get();
    //    prog_.AddLinearConstraint(lambda_.at(i) >= VectorXd::Zero(m_));
  }

  // Adding constraint for x[2] corresponding to end effector z to always be 
  // above the ground at every time step.
  // Impose constraint for timestep 1 and beyond to ensure that the plan doesn't go below the safe threshold/ground
  // but allow the 0th timestep to be where the ee currently is in order to not invalidate the x_[0] == x0 constraint.
  for (int i = 1; i < N_; i++) {
    prog_.AddLinearConstraint(x_.at(i)[2] >= options_.ee_z_state_min);
  }

  input_costs_.resize(N_);
  for (int i = 0; i < N_ + 1; i++) {
    target_cost_[i] =
        prog_
            .AddQuadraticCost(Q_.at(i) * 2, -2 * Q_.at(i) * x_desired_.at(i),
                              x_.at(i), 1)
            .evaluator()
            .get();
    if (i < N_) {
      input_costs_[i] =
          prog_.AddQuadraticCost(2 * R_.at(i), VectorXd::Zero(k_), u_.at(i), 1)
              .evaluator();
    }
  }
}

void C3::UpdateCostMatrices(const C3::CostMatrices& costs) {
  DRAKE_DEMAND(costs.Q.size() == N_ + 1);
  DRAKE_DEMAND(costs.R.size() == N_);
  DRAKE_DEMAND(costs.U.size() == N_);
  DRAKE_DEMAND(costs.G.size() == N_);
  DRAKE_DEMAND(costs.Q[0].rows() == n_);
  DRAKE_DEMAND(costs.Q[0].cols() == n_);
  DRAKE_DEMAND(costs.R[0].rows() == k_);
  DRAKE_DEMAND(costs.R[0].cols() == k_);
  DRAKE_DEMAND(costs.U[0].rows() == n_ + m_ + k_);
  DRAKE_DEMAND(costs.U[0].cols() == n_ + m_ + k_);
  DRAKE_DEMAND(costs.G[0].rows() == n_ + m_ + k_);
  DRAKE_DEMAND(costs.G[0].cols() == n_ + m_ + k_);
  Q_ = costs.Q;
  R_ = costs.R;
  U_ = costs.U;
  G_ = costs.G;
}

void C3::UpdateLCS(const LCS& lcs) {
  DRAKE_DEMAND(lcs.A_.size() == N_);
  DRAKE_DEMAND(lcs.A_[0].rows() == n_);
  DRAKE_DEMAND(lcs.A_[0].cols() == n_);
  DRAKE_DEMAND(lcs.D_[0].cols() == m_);
  DRAKE_DEMAND(lcs.B_[0].cols() == k_);
  // Update the stored LCS object.
  lcs_ = lcs;
  // lcs_(lcs);

  // first 4 lines are unnecessary
  A_ = lcs.A_;
  B_ = lcs.B_;
  D_ = lcs.D_;
  d_ = lcs.d_;
  E_ = lcs.E_;
  F_ = lcs.F_;
  H_ = lcs.H_;
  c_ = lcs.c_;
  h_is_zero_ = H_[0].isZero(0);

  MatrixXd LinEq = MatrixXd::Zero(n_, 2 * n_ + m_ + k_);
  LinEq.block(0, n_ + k_ + m_, n_, n_) = -1 * MatrixXd::Identity(n_, n_);

  auto Dn = D_.at(0).norm();
  auto An = A_.at(0).norm();
  AnDn_ = An / Dn;
  // Scale the D, E, c, and H matrices by AnDn_ to have a better conditioned
  // optimization problem.
  for (int i = 0 ; i < N_; ++i){
    D_.at(i) *= AnDn_;
    E_.at(i) /= AnDn_;
    c_.at(i) /= AnDn_;
    H_.at(i) /= AnDn_;
  }

  for (int i = 0; i < N_; i++) {
    LinEq.block(0, 0, n_, n_) = A_.at(i);
    LinEq.block(0, n_, n_, m_) = D_.at(i);
    LinEq.block(0, n_ + m_, n_, k_) = B_.at(i);

    dynamics_constraints_[i]->UpdateCoefficients(LinEq, -lcs.d_.at(i));
  }
}

void C3::UpdateCostLCS(const LCS& lcs) {
  DRAKE_DEMAND(lcs.A_.size() == N_);
  DRAKE_DEMAND(lcs.A_[0].rows() == n_);
  DRAKE_DEMAND(lcs.A_[0].cols() == n_);
  DRAKE_DEMAND(lcs.B_[0].cols() == k_);
  // Update the stored LCS object.
  if (!LCS_for_cost_computation_){
    LCS_for_cost_computation_ = std::make_unique<LCS>(lcs);
  }else{
    *LCS_for_cost_computation_ = lcs;
  }
}

void C3::UpdateTarget(const std::vector<Eigen::VectorXd>& x_des) {
  x_desired_ = x_des;
  for (int i = 0; i < N_ + 1; i++) {
    target_cost_[i]->UpdateCoefficients(Q_.at(i) * 2,
                                        -2 * Q_.at(i) * x_desired_.at(i));
  }
}

void C3::Solve(const VectorXd& x0, vector<VectorXd>& delta,
               vector<VectorXd>& w, bool verbose) {
  vector<MatrixXd> Gv = G_;

  for (int i = 0; i < N_; ++i) {
    input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
                                        -2 * R_.at(i) * u_sol_->at(i));
    // input_costs_[i]->UpdateCoefficients(2 * R_.at(i),
    //                                     Eigen::VectorXd::Zero(k_));
  }
  
  if(verbose){
    std::cout << "x0: " << x0.transpose() << std::endl;
  }

  for (int iter = 0; iter < options_.admm_iter; iter++) {
    ADMMStep(x0, &delta, &w, &Gv, iter, verbose);    
  }

  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta.at(i) - w.at(i);
  }
  zfin_ = SolveQP(x0, Gv, WD, options_.admm_iter, true);

  if(verbose){
    std::cout << "Final ADMM Iteration: " << options_.admm_iter << std::endl;
    // Make a printable delta and w matrix.
    Eigen::MatrixXd verbose_delta = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    Eigen::MatrixXd verbose_w = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    Eigen::MatrixXd verbose_zfin = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    Eigen::MatrixXd verbose_zsol = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    for(int i = 0; i < N_; i++){
      verbose_delta.col(i) = delta[i];
      verbose_w.col(i) = w[i];
      verbose_zfin.col(i) = zfin_[i];
      verbose_zsol.col(i) = z_sol_->at(i);
    }
    std::cout<<"zfin: \n"<<verbose_zfin<<std::endl;
    std::cout<<"zsol: \n"<<verbose_zsol<<std::endl;
    std::cout<<"delta: \n"<<verbose_delta<<std::endl;
    std::cout<<"w: \n"<<verbose_w<<std::endl;
  }

  *z_sol_ = delta;
  z_sol_->at(0).segment(0, n_) = x0;
  for (int i = 1; i < N_; ++i) {
    z_sol_->at(i).segment(0, n_) =
        A_.at(i - 1) * x_sol_->at(i - 1) + B_.at(i - 1) * u_sol_->at(i - 1) +
        D_.at(i - 1) * lambda_sol_->at(i - 1) + d_.at(i - 1);
  }

  *w_sol_ = w;
  *delta_sol_ = delta;

  // Unscale lambda before output for better visualizationa and interpretation.
  // These values will now be in Newtons.
  for (int i = 0; i < N_; ++i){
    lambda_sol_->at(i) *= AnDn_;
    z_sol_->at(i).segment(n_, m_) *= AnDn_;
  }
  // Will added this line since zfin_ should be z_sol? Reconfirm with him.
  // Commented for now since these look to be the same.
  // zfin_ = *z_sol_;
}

// This function relies on the previously computed zfin_ from the Solve function.
// Calculate the C3 cost and feasible trajectory associated with applying a 
// provided control input sequence to a system at a provided initial state.
// Or, use the zfin_ trajectory if cost_type is false.
std::pair<double,std::vector<Eigen::VectorXd>> C3::CalcCost(int cost_type) const{
  // Extract the locally stored state and control sequences.
  vector<VectorXd> UU(N_, VectorXd::Zero(k_));
  std::vector<Eigen::VectorXd> XX(N_+1, VectorXd::Zero(n_)); 

  // If cost_type is 0, simulate the dynamics to get the
  // full state trajectory. 
  if (cost_type == 0){
    XX[0] = zfin_[0].segment(0, n_);
    for (int i = 0; i < N_; i++){
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      if(LCS_for_cost_computation_){
        XX[i+1] = LCS_for_cost_computation_->Simulate(XX[i], UU[i]);
      }
      else{
        XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
      }
    }
  }
  else if(cost_type == 1){
    // If cost_type is 1, use the provided zfin_ trajectory for the full state.
    for (int i = 0; i < N_; i++){
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      XX[i] = zfin_[i].segment(0, n_);
      if(i == N_-1){
        if(LCS_for_cost_computation_){
          XX[i+1] = LCS_for_cost_computation_->Simulate(XX[i], UU[i]);
        }
        else{
          XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
        }
      }
    }
  }
  else if(cost_type == 2){
    // If cost_type is 2, use z_fin for ee trajectory but simulate forward for 
    // the object trajectory.
    // Simulate the object trajectory.
    XX[0] = zfin_[0].segment(0, n_);
    for (int i = 0; i < N_; i++){
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      if(LCS_for_cost_computation_){
        XX[i+1] = LCS_for_cost_computation_->Simulate(XX[i], UU[i]);
      }
      else{
        XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
      }
    }
    // Replace ee traj with those from zfin_.
    for (int i = 0; i < N_; i++){
      XX[i].segment(0,3) = zfin_[i].segment(0,3);
      // This is replacing the XX[N_] state of the end effector to a state
      // simulated from the last control input in UU and last end effector state
      // from the C3 plan. This is different from what is already set in the 
      // previous loop because the [N_-1] states of the end effector are not
      // the same.
      if(i == N_-1){
        if(LCS_for_cost_computation_){
          XX[i+1].segment(0,3) = LCS_for_cost_computation_->Simulate(XX[i], UU[i]).segment(0,3);
        }
        else{
          XX[i+1].segment(0,3) = lcs_.Simulate(XX[i], UU[i]).segment(0,3);
        }
      }
    }
  }
  else if(cost_type == 3){
    // If cost_type is 3, try to emulate the real cost of the system associated 
    // not only applying the u from the zfin_[0] but also the u associated with 
    // tracking the position plan over time.
    // u = zfin_[0].segment(n_ + m_, k_) + Kp*(x_desired_[0] - zfin_[0].segment(0, 3));
    vector<VectorXd> UU_new(N_, VectorXd::Zero(k_));
    std::vector<Eigen::VectorXd> XX_new(N_+1, VectorXd::Zero(n_));
    // Set the UU and XX values to the z_fin values first for the emulated PD 
    // controller to have for error computation in place of all 0s.
    for (int i = 0; i < N_; i++){
      UU[i] = zfin_[i].segment(n_ + m_, k_);
      XX[i] = zfin_[i].segment(0, n_);
      if(i == N_-1){
        if(LCS_for_cost_computation_){
          XX[i+1] = LCS_for_cost_computation_->Simulate(XX[i], UU[i]);
        }
        else{
          XX[i+1] = lcs_.Simulate(XX[i], UU[i]);
        }
      }
    }

    // Set the PD gains for the emulated tracking controller.
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Identity(3,3);

    XX_new[0] = zfin_[0].segment(0, n_);
    std::cout<<"XX_new[0]: "<<XX_new[0].transpose()<<std::endl;
    // This will just be the original u from zfin_[0] for the first time step.

    for (int i = 0; i < N_; i++){
      UU_new[i] = UU[i] + 
        Kp*(XX[i].segment(0, 3) - XX_new[i].segment(0, 3)) + 
        Kd*(XX[i].segment(10, 3) - XX_new[i].segment(10, 3));
        if(LCS_for_cost_computation_){
          XX_new[i+1] = LCS_for_cost_computation_->Simulate(XX_new[i], UU_new[i]);
        }
        else{
          XX_new[i+1] = lcs_.Simulate(XX_new[i], UU_new[i]);
        }
      std::cout<<"XX_new["<<i+1<<"]: "<<XX_new[i+1].transpose()<<std::endl;
    }
    // Replace the original state and control sequences with the new ones for 
    // the cost calculation.
    XX = XX_new;
    UU = UU_new;   
  }

  // Declare Q_eff and R_eff as the Q and R to use for cost computation.
  std::vector<Eigen::MatrixXd> Q_eff = Q_;
  std::vector<Eigen::MatrixXd> R_eff = R_;

  // Calculate the cost over the N+1 time steps.
  double cost = 0;
  // Print full Q_eff and R_eff matrices.
  // std::cout<<"\tQ_eff at i = 0: "<<std::endl;
  // std::cout<<Q_eff.at(0)<<std::endl;
  // std::cout<<"\tR_eff at i = 0: "<<std::endl;
  // std::cout<<R_eff.at(0)<<std::endl;

  for (int i = 0; i < N_; i++){
    // Print cost contribution for each part of the cost function.
    // std::cout<<"\ttimestep "<<i<<std::endl;
    // std::cout<<"\t\tCost contribution from qeex, qeey, qeez: "<<
    //   (XX[i].segment(0,3) - x_desired_[i].segment(0,3)).transpose()*
    //   Q_eff.at(i).block(0,0,3,3)*(XX[i].segment(0,3) - x_desired_[i].segment(0,3))<<std::endl;
    //   // Print x and x_desired values for this part of the cost function.
    // std::cout<<"\t\tx_ee: "<<XX[i].segment(0,3).transpose()<<std::endl;
    // std::cout<<"\t\tx_desired_ee: "<<x_desired_[i].segment(0,3).transpose()<<std::endl;
    // // Print error terms and cost values for this part of the cost function.
    // std::cout<<"\t\txee error: "<<(XX[i].segment(0,3) - x_desired_[i].segment(0,3)).transpose()<<std::endl;
    // if (i == 0) {
    //   std::cout<<"Unscaled object position error: "<<
    //     (XX[i].segment(7,3) - x_desired_[i].segment(7,3)).transpose()*  //Q_eff.at(i).block(7,7,3,3)*
    //     (XX[i].segment(7,3) - x_desired_[i].segment(7,3))<<std::endl;
    //   std::cout<<"\tUnscaled object orientation error: "<<
    //     (XX[i].segment(3,4) - x_desired_[i].segment(3,4)).transpose()*  //Q_eff.at(i).block(3,3,4,4)*
    //     (XX[i].segment(3,4) - x_desired_[i].segment(3,4))<<std::endl;
    //   // std::cout<<"\t\tcost contribution from vxee, vyee, vzee: "<<
    //   //   (XX[i].segment(10,3) - x_desired_[i].segment(10,3)).transpose()*
    //   //   Q_eff.at(i).block(10,10,3,3)*(XX[i].segment(10,3) - x_desired_[i].segment(10,3))<<std::endl;
    //   std::cout<<"\tUnscaled object angular velocity error: "<<
    //     (XX[i].segment(13,3) - x_desired_[i].segment(13,3)).transpose()* //Q_eff.at(i).block(13,13,3,3)*
    //     (XX[i].segment(13,3) - x_desired_[i].segment(13,3))<<std::endl;

      // if ((XX[i].segment(3,4) - x_desired_[i].segment(3,4)).transpose()*  //Q_eff.at(i).block(3,3,4,4)*
      //   (XX[i].segment(3,4) - x_desired_[i].segment(3,4)) > 0.99) {
      //     std::cout<<"\nWARNING! ISSUE DETECTED:"<<std::endl;
      //     std::cout<<"Cost type: "<<cost_type<<std::endl;
      //     std::cout<<"XX[0] = "<<XX[0].transpose()<<std::endl;
      //     std::cout<<"x_desired_[0] = "<<x_desired_[0].transpose()<<std::endl;
      //     std::cout<<"zfin_[0] = "<<zfin_[0].transpose()<<std::endl;
      //     std::cout<<"\n"<<std::endl;
      // }
    // }

    // print object angular velocity error
    // std::cout<<"Cost contribution from angular velocity error: "<<
    //   (XX[i].segment(13,3) - x_desired_[i].segment(13,3)).transpose()*(XX[i].segment(13,3) - x_desired_[i].segment(13,3))<<std::endl;
    // std::cout<<"Cost contribution from object position error: "<<
    //   (XX[i].segment(7,3) - x_desired_[i].segment(7,3)).transpose()*(XX[i].segment(7,3) - x_desired_[i].segment(7,3))<<std::endl;

    // std::cout<<"\t\tcost contribution from vxox, vxoy, vxoz: "<<
    //   (XX[i].segment(16,3) - x_desired_[i].segment(16,3)).transpose()*
    //   Q_eff.at(i).block(16,16,3,3)*(XX[i].segment(16,3) - x_desired_[i].segment(16,3))<<std::endl;

    // std::cout<<"\t\tCost contribution from u: "<<
    //   UU[i].transpose()*R_eff.at(i)*UU[i]<<std::endl;
    cost = cost + 
      (XX[i] - x_desired_[i]).transpose()*Q_eff.at(i)*(XX[i] - x_desired_[i]) + 
      UU[i].transpose()*R_eff.at(i)*UU[i];
  }
  cost = cost + 
    (XX[N_] - x_desired_[N_]).transpose()*Q_eff.at(N_)*(XX[N_] - x_desired_[N_]);

  // Return the cost and the rolled out state trajectory.
  std::pair <double, std::vector<VectorXd>> ret (cost, XX);
  return ret;
}

void C3::ADMMStep(const VectorXd& x0, vector<VectorXd>* delta,
                  vector<VectorXd>* w, vector<MatrixXd>* Gv,
                  int admm_iteration, bool verbose) {
  vector<VectorXd> WD(N_, VectorXd::Zero(n_ + m_ + k_));

  for (int i = 0; i < N_; i++) {
    WD.at(i) = delta->at(i) - w->at(i);
  }

  vector<VectorXd> z = SolveQP(x0, *Gv, WD, admm_iteration, true);
  if(verbose){
    std::cout << "SolveQP Iteration: " << admm_iteration << std::endl;
    Eigen::MatrixXd verbose_z = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    for(int i = 0; i < N_; i++){
      verbose_z.col(i) = z[i];
    }
    std::cout<<"z: \n"<<verbose_z<<std::endl;
  }

  vector<VectorXd> ZW(N_, VectorXd::Zero(n_ + m_ + k_));
  for (int i = 0; i < N_; i++) {
    ZW[i] = w->at(i) + z[i];
  }
  if (U_[0].isZero(0)) {
    *delta = SolveProjection(*Gv, ZW, admm_iteration);

  } else {
    *delta = SolveProjection(U_, ZW, admm_iteration);
  }
  if(verbose){
    std::cout << "ADMM Iteration: " << admm_iteration << std::endl;
    Eigen::MatrixXd verbose_delta = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    // Eigen::MatrixXd verbose_w = Eigen::MatrixXd::Zero(n_ + m_ + k_, N_);
    for(int i = 0; i < N_; i++){
      verbose_delta.col(i) = delta->at(i);
      // verbose_w.col(i) = w[i];
    }
    std::cout<<"delta: \n"<<verbose_delta.block(10, 0, 9, 5)<<std::endl;
  }

  for (int i = 0; i < N_; i++) {
    w->at(i) = w->at(i) + z[i] - delta->at(i);
    w->at(i) = w->at(i) / options_.rho_scale;
    Gv->at(i) = Gv->at(i) * options_.rho_scale;
  }
}

vector<VectorXd> C3::SolveQP(const VectorXd& x0, const vector<MatrixXd>& G,
                             const vector<VectorXd>& WD, int admm_iteration,
                             bool is_final_solve) {
  for (auto& constraint : constraints_) {
    prog_.RemoveConstraint(constraint);
  }
  constraints_.clear();
  if(x0[2] < options_.ee_z_state_min){
    std::cout<<"CAUTION: Initial state is below the min z height. C3 plan will have the z go up suddenly in the first step of the plan."<<std::endl;
  }
  constraints_.push_back(prog_.AddLinearConstraint(x_[0] == x0));

  if (h_is_zero_ == 1) {
    drake::solvers::MobyLCPSolver<double> LCPSolver;
    VectorXd lambda0;
    LCPSolver.SolveLcpLemkeRegularized(F_[0], E_[0] * x0 + c_[0], &lambda0);
    constraints_.push_back(prog_.AddLinearConstraint(lambda_[0] == lambda0));
  }

  for (auto& cost : costs_) {
    prog_.RemoveCost(cost);
  }
  costs_.clear();

  for (int i = 0; i < N_ + 1; i++) {
    if (i < N_) {
      costs_.push_back(prog_.AddQuadraticCost(
          2 * G.at(i).block(0, 0, n_, n_),
          -2 * G.at(i).block(0, 0, n_, n_) * WD.at(i).segment(0, n_), x_.at(i),
          1));
      costs_.push_back(prog_.AddQuadraticCost(
          2 * G.at(i).block(n_, n_, m_, m_),
          -2 * G.at(i).block(n_, n_, m_, m_) * WD.at(i).segment(n_, m_),
          lambda_.at(i), 1));
      costs_.push_back(
          prog_.AddQuadraticCost(2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_),
                                 -2 * G.at(i).block(n_ + m_, n_ + m_, k_, k_) *
                                     WD.at(i).segment(n_ + m_, k_),
                                 u_.at(i), 1));
    }
  }

  //  /// initialize decision variables to warm start
  if (warm_start_) {
    prog_.SetInitialGuess(x_[0], x0);
    for (int i = 0; i < N_ - 1; i++) {
      prog_.SetInitialGuess(x_[i], warm_start_x_[admm_iteration][i + 1]);
      prog_.SetInitialGuess(lambda_[i],
                            warm_start_lambda_[admm_iteration][i + 1]);
      prog_.SetInitialGuess(u_[i], warm_start_u_[admm_iteration][i + 1]);
    }
    prog_.SetInitialGuess(x_[N_], warm_start_x_[admm_iteration][N_]);
  }

  MathematicalProgramResult result = osqp_.Solve(prog_);

  if (result.is_success()) {
    for (int i = 0; i < N_; i++) {
      if (is_final_solve) {
        x_sol_->at(i) = result.GetSolution(x_[i]);
        lambda_sol_->at(i) = result.GetSolution(lambda_[i]);
        u_sol_->at(i) = result.GetSolution(u_[i]);
      }
      z_sol_->at(i).segment(0, n_) = result.GetSolution(x_[i]);
      z_sol_->at(i).segment(n_, m_) = result.GetSolution(lambda_[i]);
      z_sol_->at(i).segment(n_ + m_, k_) = result.GetSolution(u_[i]);

      if (warm_start_) {
        // update warm start parameters
        warm_start_x_[admm_iteration][i] = result.GetSolution(x_[i]);
        warm_start_lambda_[admm_iteration][i] = result.GetSolution(lambda_[i]);
        warm_start_u_[admm_iteration][i] = result.GetSolution(u_[i]);
      }
    }
    if (warm_start_) {
      warm_start_x_[admm_iteration][N_] = result.GetSolution(x_[N_]);
    }
  }
  else {
    std::cout<<"CAUTION: C3 QP solve did not succeed" << std::endl;
  }

  return *z_sol_;
}

void C3::AddLinearConstraint(Eigen::RowVectorXd& A, double lower_bound,
                             double upper_bound, int constraint) {
  if (constraint == 1) {
    for (int i = 1; i < N_; i++) {
      user_constraints_.push_back(
          prog_.AddLinearConstraint(A, lower_bound, upper_bound, x_.at(i)));
    }
  }

  if (constraint == 2) {
    for (int i = 0; i < N_; i++) {
      user_constraints_.push_back(
          prog_.AddLinearConstraint(A, lower_bound, upper_bound, u_.at(i)));
    }
  }

  if (constraint == 3) {
    for (int i = 0; i < N_; i++) {
      user_constraints_.push_back(prog_.AddLinearConstraint(
          A, lower_bound, upper_bound, lambda_.at(i)));
    }
  }
}

void C3::RemoveConstraints() {
  for (auto& userconstraint : user_constraints_) {
    prog_.RemoveConstraint(userconstraint);
  }
  user_constraints_.clear();
}

vector<VectorXd> C3::SolveProjection(const vector<MatrixXd>& G,
                                     vector<VectorXd>& WZ, int admm_iteration) {
  vector<VectorXd> deltaProj(N_, VectorXd::Zero(n_ + m_ + k_));
  int i;

  if (options_.num_threads > 0) {
    omp_set_dynamic(0);  // Explicitly disable dynamic teams
    omp_set_num_threads(options_.num_threads);  // Set number of threads
    omp_set_nested(1);
  }

#pragma omp parallel for num_threads(options_.num_threads)
  for (i = 0; i < N_; i++) {
    if (warm_start_) {
      if (i == N_ - 1) {
        deltaProj[i] = SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i],
                                             c_[i], admm_iteration, -1);
      } else {
        deltaProj[i] = SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i],
                                             c_[i], admm_iteration, i + 1);
      }
    } else {
      deltaProj[i] = SolveSingleProjection(G[i], WZ[i], E_[i], F_[i], H_[i],
                                           c_[i], admm_iteration, -1);
    }
  }

  return deltaProj;
}

std::vector<Eigen::VectorXd> C3::GetWarmStartX() const {
  return warm_start_x_[0];
}

std::vector<Eigen::VectorXd> C3::GetWarmStartLambda() const {
  return warm_start_lambda_[0];
}

std::vector<Eigen::VectorXd> C3::GetWarmStartU() const {
  return warm_start_u_[0];
}

}  // namespace solvers
}  // namespace dairlib
