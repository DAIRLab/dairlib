#include "systems/controllers/clqr_controller.h"

namespace dairlib{
namespace systems{

ClqrController::ClqrController(RigidBodyPlant<double>* plant,
                               VectorXd x0,
                               VectorXd u0,
                               VectorXd lambda,
                               MatrixXd J_collision,
                               int num_positions,
                               int num_velocities,
                               int num_efforts,
                               MatrixXd Q,
                               MatrixXd R,
                               double fixed_point_tolerance):
  AffineController(num_positions, num_velocities, num_efforts),
  tree_(plant->get_rigid_body_tree()), plant_(plant), x0_(x0),
  u0_(u0), lambda_(lambda), J_collision_(J_collision),
  num_positions_(num_positions),  num_velocities_(num_velocities),
  num_states_(num_positions + num_velocities),
  num_efforts_(num_efforts), Q_(Q), R_(R),
  fixed_point_tolerance_(fixed_point_tolerance)
{

  F_ = computeF();
  K_ = computeK();
  
}

MatrixXd ClqrController::computeF() {
    
  KinematicsCache<double> k_cache = tree_.doKinematics(
      x0_.head(num_positions_), x0_.tail(num_velocities_));

  //Computing the constraint jacobian
  MatrixXd J_constraint = tree_.positionConstraintsJacobian(k_cache);

  MatrixXd J(J_constraint.rows() + J_collision_.rows(), J_constraint.cols());
  J << J_constraint, 
       J_collision_;


  //std::cout << "J times x----------------------------" << std::endl;
  //std::cout << J.rows() << " " << J.cols() << " " << x0_.size() << std::endl;
  //std::cout << J*x0_.head(num_positions_) << std::endl;

  const int r = J.rows();
  const int c = J.cols();

  // j_dot is zero for the time invariant case
  MatrixXd F =  MatrixXd::Zero(2*r, 2*c);

  //Computing F by populating with J
  F.block(0, 0, r, c) = J;
  F.block(r, c, r, c) = J;


  //std::cout << "F times x----------------------------" << std::endl;
  //std::cout << F*x0_ << std::endl;


  return F;

}

MatrixXd ClqrController::computeK() {

  //Finding the null space
  HouseholderQR<MatrixXd> qr(F_.transpose());
  MatrixXd Q = qr.householderQ();
  
  MatrixXd P = Q.block(0, F_.rows(), Q.rows(), Q.cols() - F_.rows());
  P.transposeInPlace();

  auto context = plant_->CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);
  context->FixInputPort(0, std::make_unique<systems::BasicVector<double>>(u0_));

  //Linearizing about the operating point
  
  MatrixXd A(num_states_, num_states_);
  MatrixXd B(num_states_, num_efforts_);

  // Generating an autodiff vector with all the stacked variables
  VectorXd xu0(num_states_ + num_efforts_);
  xu0 << x0_, u0_;
  auto xu0_autodiff = initializeAutoDiff(xu0);

  // Extracting the x and u values
  auto x0_autodiff = xu0_autodiff.head(num_states_);
  auto u0_autodiff = xu0_autodiff.tail(num_efforts_);

  // Lambda being initialized as a separate autodiff means that
  // the gradient with respect to lambda will not be computed
  auto lambda_autodiff = initializeAutoDiff(lambda_);

  auto x_dot0_autodiff = CalcTimeDerivativesUsingLambda<AutoDiffXd>(
      tree_, x0_autodiff, u0_autodiff, lambda_autodiff); 

  // Making sure that the derivative is zero (fixed point)
  VectorXd x_dot0 = autoDiffToValueMatrix(x_dot0_autodiff);
  
  if (!x_dot0.isZero(fixed_point_tolerance_)) {
    
    throw std::runtime_error(
        "The nominal operating point (x0, u0) is not an equilibrium point "
        "of the system.");
  }


  const MatrixXd AB = 
    autoDiffToGradientMatrix(x_dot0_autodiff);
  //std::cout << "AB: " << std::endl;
  //std::cout << AB << std::endl;
  //std::cout << AB.rows() << " " << AB.cols() << std::endl;

  A = AB.leftCols(num_states_);
  B = AB.rightCols(num_efforts_);


  //auto linear_system = Linearize(*plant_, *context, 0, kNoOutput);
  MatrixXd A_new_coord = P*A*P.transpose();
  MatrixXd B_new_coord = P*B;

  //Computing LQR result
  auto lqr_result = LinearQuadraticRegulator(A_new_coord, B_new_coord, Q_, R_);
  return lqr_result.K*P;

}


}//namespace systems
}//namespace dairlib

