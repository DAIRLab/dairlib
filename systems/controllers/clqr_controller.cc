#include "systems/controllers/clqr_controller.h"

namespace dairlib{
namespace systems{

ClqrController::ClqrController(const RigidBodyPlant<double>& plant,
                               const RigidBodyPlant<AutoDiffXd>& plant_autodiff,
                               VectorXd x0,
                               VectorXd u0,
                               VectorXd lambda0,
                               MatrixXd Q,
                               MatrixXd R,
                               double fixed_point_tolerance):
  AffineController(plant.get_num_positions(),
                   plant.get_num_velocities(),
                   plant.get_num_actuators()),
  tree_(plant.get_rigid_body_tree()), plant_(plant),
  plant_autodiff_(plant_autodiff), x0_(x0),
  u0_(u0), lambda0_(lambda0), 
  num_positions_(plant.get_num_positions()),
  num_velocities_(plant.get_num_velocities()),
  num_states_(plant.get_num_states()),
  num_efforts_(plant.get_num_actuators()), Q_(Q), R_(R),
  fixed_point_tolerance_(fixed_point_tolerance)
{

  F_ = computeF();
  K_ = computeK();
  
}

MatrixXd ClqrController::computeF() {
    
  KinematicsCache<double> k_cache = tree_.doKinematics(
      x0_.head(num_positions_), x0_.tail(num_velocities_));

  //Computing the constraint jacobian
  MatrixXd J_tree = tree_.positionConstraintsJacobian(k_cache);

  // Contact Jacobian
  CassiePlant<double> cassie_plant(plant_);
  MatrixXd J_contact;

  if (num_positions_ == 22) {
    J_contact = cassie_plant.CalcContactJacobianCassie(x0_.head(num_positions_), 
                                                      x0_.tail(num_velocities_),
                                                      lambda0_.size() - J_tree.rows());
  }

  MatrixXd J(J_tree.rows() + J_contact.rows(), J_tree.cols());
  J << J_tree, 
       J_contact;

  const int r = J.rows();
  const int c = J.cols();

  // j_dot is zero for the time invariant case
  MatrixXd F =  MatrixXd::Zero(2*r, 2*c);

  //Computing F by populating with J
  F.block(0, 0, r, c) = J;
  F.block(r, c, r, c) = J;


  return F;

}

MatrixXd ClqrController::computeK() {

  //Finding the null space
  HouseholderQR<MatrixXd> qr(F_.transpose());
  MatrixXd Q = qr.householderQ();
  
  MatrixXd P = Q.block(0, F_.rows(), Q.rows(), Q.cols() - F_.rows());
  P.transposeInPlace();

  auto context = plant_.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);
  context->FixInputPort(0, std::make_unique<systems::BasicVector<double>>(u0_));

  //Linearizing about the operating point
  
  MatrixXd A;
  MatrixXd B;

  // Generating an autodiff vector with all the stacked variables
  VectorXd x_u_l0(num_states_ + num_efforts_ + lambda0_.size());
  x_u_l0 << x0_, u0_, lambda0_;
  auto x_u_l0_autodiff = initializeAutoDiff(x_u_l0);

  // Extracting the x and u values
  auto x0_autodiff = x_u_l0_autodiff.head(num_states_);
  auto u0_autodiff = x_u_l0_autodiff.segment(num_states_, num_efforts_);
  auto lambda0_autodiff = x_u_l0_autodiff.tail(lambda0_.size());

  CassiePlant<AutoDiffXd> cassie_plant(plant_autodiff_);
  VectorX<AutoDiffXd> x_dot0_autodiff;

  if (num_positions_ == 22) {
  x_dot0_autodiff = cassie_plant.CalcTimeDerivativesCassieStanding(
      x0_autodiff, u0_autodiff, lambda0_autodiff); 
  }else {
  x_dot0_autodiff = cassie_plant.CalcTimeDerivativesCassie(
        x0_autodiff, u0_autodiff, lambda0_autodiff); 
  }


  // Making sure that the derivative is zero (fixed point)
  VectorXd x_dot0 = autoDiffToValueMatrix(x_dot0_autodiff);
  
  //if (!x_dot0.isZero(fixed_point_tolerance_)) {

  //  std::cout << "********* x dot *********" << std::endl;
  //  std::cout << x_dot0 << std::endl;
  //  std::cout << "Tolerance: " << fixed_point_tolerance_ << std::endl;
  //  
  //  throw std::runtime_error(
  //      "The nominal operating point (x0, u0) is not an equilibrium point "
  //      "of the system.");
  //}


  const MatrixXd AB = 
    autoDiffToGradientMatrix(x_dot0_autodiff);

  A = AB.leftCols(num_states_);
  B = AB.block(
      0, num_states_, AB.rows(), num_efforts_);

  DRAKE_DEMAND(A.rows() == num_states_);
  DRAKE_DEMAND(A.cols() == num_states_);
  DRAKE_DEMAND(B.rows() == num_states_);
  DRAKE_DEMAND(B.cols() == num_efforts_);

  //auto linear_system = Linearize(*plant_, *context, 0, kNoOutput);
  MatrixXd A_new_coord = P*A*P.transpose();
  MatrixXd B_new_coord = P*B;

  //Computing LQR result
  auto lqr_result = LinearQuadraticRegulator(A_new_coord, B_new_coord, Q_, R_);
  return lqr_result.K*P;

}


}//namespace systems
}//namespace dairlib

