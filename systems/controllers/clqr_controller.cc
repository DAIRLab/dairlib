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
    VectorXd q = x0_.head(num_positions_);
    VectorXd v = x0_.tail(num_velocities_);
    
    KinematicsCache<double> kcache = tree_.doKinematics(q, v);
    Matrix<double, Dynamic, Dynamic> c_jac = tree_.positionConstraintsJacobian(kcache);

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
    VectorX<double> cstate = xd_;
    std::cout << x0_.transpose() << std::endl;
    std::cout << xd_.transpose() << std::endl;
    VectorX<double> zero_input = VectorXd::Zero(num_efforts_);
    context->set_continuous_state(std::make_unique<ContinuousState<double>>(BasicVector<double>(cstate).Clone(), num_positions_, num_velocities_, 0));
    context->FixInputPort(0, std::make_unique<systems::BasicVector<double>>(zero_input));
    auto linear_system = Linearize(*plant_, *context, 0, kNoOutput);

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

    







