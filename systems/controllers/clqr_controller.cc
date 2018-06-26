#include "systems/controllers/clqr_controller.h"

namespace dairlib{
namespace systems{

ClqrController::ClqrController(RigidBodyPlant<double>* plant, VectorXd x0, VectorXd xd, int num_positions, int num_velocities, int num_efforts, Matrix<double, Dynamic, Dynamic> Q, Matrix<double, Dynamic, Dynamic> R): LinearController(num_positions, num_velocities, num_efforts), tree_(plant->get_rigid_body_tree()), plant_(plant), x0_(x0), xd_(xd), num_positions_(num_positions), num_velocities_(num_velocities), num_states_(num_positions + num_velocities), num_efforts_(num_efforts), Q_(Q), R_(R)
{
    F_ = computeF();
    K_ = computeK();

}

Matrix<double, Dynamic, Dynamic> ClqrController::computeF()
{
    
    KinematicsCache<double> kcache_0 = tree_.doKinematics(x0_.head(num_positions_), x0_.tail(num_velocities_));
    Matrix<double, Dynamic, Dynamic> c_jac = tree_.positionConstraintsJacobian(kcache_0);

    const int r = c_jac.rows();
    const int c = c_jac.cols();

    // j_dot is zero for the time invariant case

    Matrix<double, Dynamic, Dynamic> F =  MatrixXd::Zero(2*r, 2*c);

    F.block(0, 0, r, c) = c_jac;
    F.block(r, c, r, c) = c_jac;

    return F;

}

Matrix<double, Dynamic, Dynamic> ClqrController::computeK()
{

    //Finding the null space

    HouseholderQR<Matrix<double, Dynamic, Dynamic>> qr(F_.transpose());
    Matrix<double, Dynamic, Dynamic> Q = qr.householderQ();
    
    Matrix<double, Dynamic, Dynamic> P = Q.block(0, F_.rows(), Q.rows(), Q.cols() - F_.rows());
    P.transposeInPlace();
    

    //Linearizing
    auto context = plant_->CreateDefaultContext();

    //Computing inputs for the stabilizable point
    Matrix<double, Dynamic, Dynamic> B = tree_.B;
    Matrix<double, Dynamic, Dynamic> B_pinv = B.completeOrthogonalDecomposition().pseudoInverse();
    KinematicsCache<double> kcache_d = tree_.doKinematics(xd_.head(num_positions_), xd_.tail(num_velocities_));
    const typename RigidBodyTree<double>::BodyToWrenchMap no_wrenches;
    VectorXd C = tree_.dynamicsBiasTerm(kcache_d, no_wrenches);
    VectorX<double> u0 = B.colPivHouseholderQr().solve(C);

    //std::cout << C << std::endl;
    //std::cout << C - B*u0 << std::endl;
    //std::cout << num_positions_ << " " << num_velocities_ << " " << num_states_ << std::endl;
    context->set_continuous_state(std::make_unique<ContinuousState<double>>(BasicVector<double>(xd_).Clone(), num_positions_, num_velocities_, 0));
    context->FixInputPort(0, std::make_unique<systems::BasicVector<double>>(u0));
    //auto linear_system = Linearize(*plant_, *context, 0, kNoOutput);

    //auto lqr_result = LinearQuadraticRegulator(P*linear_system->A()*P.transpose(), P*linear_system->B(), Q_, R_);
    //K_ = lqr_result.K*P;

    return P;

}


int ClqrController::getNumPositions()
{
    return num_positions_;
}

int ClqrController::getNumVelocities()
{
    return num_velocities_;
}

int ClqrController::getNumStates()
{
    return num_states_;
}

int ClqrController::getNumEfforts()
{
    return num_efforts_;
}

Matrix<double, Dynamic, Dynamic> ClqrController::getQ()
{
    return Q_;
}

Matrix<double, Dynamic, Dynamic> ClqrController::getR()
{
    return R_;
}

Matrix<double, Dynamic, Dynamic> ClqrController::getK()
{
    return K_;
}

void ClqrController::setK(Matrix<double, Dynamic, Dynamic> K)
{
    K_ = K;
}

void ClqrController::setQ(Matrix<double, Dynamic, Dynamic> Q)
{
    Q_ = Q;
}

void ClqrController::setR(Matrix<double, Dynamic, Dynamic> R)
{
    R_ = R;
}



}//namespace systems
}//namespace dairlib

    







