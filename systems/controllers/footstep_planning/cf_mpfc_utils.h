#pragma once
#include "alip_utils.h"
#include "drake/math/autodiff_gradient.h"


namespace dairlib::systems::controllers::cf_mpfc_utils {

static constexpr Eigen::Index SrbDim = 12;
static constexpr int theta_idx = 0;
static constexpr int com_idx = 3;
static constexpr int w_idx = 6;
static constexpr int com_dot_idx = 9;



/**
 * State is in this order:
 * R_x (rotation matrix x column)
 * R_y (rotation matrix y column)
 * R_z (rotation matrix z column)
 * c (center of mass pos)
 * ω (angular velocity of ACOM, expressed in an inertial frame)
 * ċ (com velocity)
 *
 */
template <typename T>
using CentroidalState = Eigen::Matrix<T, SrbDim, 1>;

template <typename T>
using CentroidalStateDeriv = Eigen::Matrix<T, SrbDim, 1>;


drake::multibody::RotationalInertia<double> CalcRotationalInertiaAboutCoM(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& plant_context,
    drake::multibody::ModelInstanceIndex model_instance,
    const drake::multibody::Frame<double>& floating_base_frame);

/**
 * Calculate the robot's single rigid body state in the current stance frame.
 *
 * The stance frame is defined as an inertial frame with identity rotation from
 * the pelvis yaw frame, with it's origin located at the current stance foot
 * position
 *
 * @param plant the plant
 * @param plant_context up-to-date context with the plant's state
 * @param acom_function a function which calculates the ACOM orientation
 * (relative to the world frame) give the plant and context
 * @param stance_foot current stance foot
 * @return Centroidal State - vectorized rotation matrix,
 *
 * Templated to allow for linearizing about an operating point with AutoDiff.
 * on an M1 mac with 1 contact point.
 *
 */

// TODO (@Brian-Acosta) may need to implement analytical gradients of these
//   dynamics for speed
CentroidalState<double> GetCentroidalState(
    const drake::multibody::MultibodyPlant<double> &plant,
    const drake::systems::Context<double> &plant_context,
    drake::multibody::ModelInstanceIndex model_instance,
    std::function<Eigen::Matrix3d(
        const drake::multibody::MultibodyPlant<double> &,
        const drake::systems::Context<double> &)> acom_function,
    const alip_utils::PointOnFramed &stance_foot);

void LinearizeSRBDynamics(
    const CentroidalState<double>& x,
    const std::vector<Eigen::Vector3d>& contact_points,
    const std::vector<Eigen::Vector3d>& contact_forces,
    const Eigen::Matrix3d& I, double m,
    Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& c);

/*!
 * TODO (@Brian-Acosta) add df/dh
 *
 * Linearizes a trapezoidal direct collocation constraint into the form
 * A [x0 x1] + B [u0 u1] = b by first linearizing the continuous dynamics,
 * then making a trapezoidal approximation of integrating the dynamics
 *
 * @param x0 state at knot 0
 * @param x1 state at knot 1
 * @param u0 input at knot 0
 * @param u1 input at knot 1
 * @param A df_dx
 * @param B df_du
 * @param b drift term
 */
void LinearizeDirectCollocationConstraint(
    double h,
    const CentroidalState<double>& x0, const CentroidalState<double>& x1,
    const Eigen::VectorXd& u0, const Eigen::VectorXd& u1,
    const std::vector<Eigen::Vector3d>& contact_points,
    const Eigen::Matrix3d& I, double m,
    Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& b);

template <typename T>
CentroidalStateDeriv<T> SRBDynamics(
    const CentroidalState<T>& state,
    const std::vector<drake::Vector3<T>>& contact_forces,
    const std::vector<Eigen::Vector3d>& contact_points,
    const Eigen::Matrix3d& I, double m);

template <typename T>
drake::Vector4<T> CalculateReset(
    const CentroidalState<T>& x_pre,
    const drake::Vector3<T>& p_pre, const drake::Vector3<T>& p_post,
    const Eigen::Matrix3d& I, double m);

// TODO (@Brian-Acosta) Should we swap the order here? Do we need the gradient
//  w.r.t p_pre ?
void LinearizeReset(const CentroidalState<double>& x_pre,
                    const Eigen::Vector3d& p_pre, const Eigen::Vector3d& p_post,
                    const Eigen::Matrix3d& I, double m,
                    Eigen::MatrixXd& Ax, Eigen::MatrixXd& B_post, Eigen::Vector4d& b);

}