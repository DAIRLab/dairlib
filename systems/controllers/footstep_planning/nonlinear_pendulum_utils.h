#pragma once
#include "alip_utils.h"

namespace dairlib::systems::controllers::nonlinear_pendulum {

using drake::Vector6d;
using Eigen::Vector2d;

constexpr int theta_y_idx = 0;
constexpr int theta_x_idx = 1;
constexpr int r_idx = 2;
constexpr int l_y_idx = 3;
constexpr int l_x_idx = 4;
constexpr int rdot_idx = 5;


/*!
 * Calculates the pendulum state [theta_y, theta_x, r, L_y, L_x, r_dot]
 */
Vector6d CalcPendulumState(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    alip_utils::PointOnFramed stance_foot, std::string floating_base_body);

void LinearizeTrapezoidalCollocationConstraint(
    double h, const Vector6d& x0, const Vector6d& x1, const Vector2d& u0,
    const Vector2d& u1, double m,
    Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b);

/*!
 *
 * x+ = g(x) --> x+ = A(x - x*) + B(p - p*) + b
 */
void LinearizeALIPReset(
    const Vector6d& x, const Eigen::Vector3d& p_pre,
    const Eigen::Vector3d& p_post, double m,
    Eigen::MatrixXd& Ax, Eigen::MatrixXd& Bp, Eigen::Vector4d& b);

/*
 * Input u = [ankle_torque, rddot];
 */
template <typename T>
drake::Vector6<T> CalcPendulumDynamics(
    const drake::Vector6<T>& x, const drake::Vector2<T>& u, double m);

template <typename T>
drake::Vector4<T> CalcALIPReset(
    const drake::Vector6<T>& x_pre, const Eigen::Vector3d& p_pre,
    const drake::Vector3<T>& p_post, double m);

}