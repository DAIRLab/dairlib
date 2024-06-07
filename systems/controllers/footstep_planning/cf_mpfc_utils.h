#include "alip_utils.h"
#include "drake/math/autodiff_gradient.h"


namespace dairlib::systems::controllers::cf_mpfc_utils {

static constexpr Eigen::Index SrbDim = 18;

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
    const drake::multibody::Frame<double> &floating_base_frame,
    const std::function<Eigen::Matrix3d(
        const drake::multibody::MultibodyPlant<double> &,
        const drake::systems::Context<double> &)> &acom_function,
    const alip_utils::PointOnFramed &stance_foot);

void LinearizeSRBDynamics(
    const CentroidalState<double>& x,
    const std::vector<Eigen::Vector3d>& contact_points,
    const std::vector<Eigen::Vector3d>& contact_forces,
    const Eigen::Matrix3d& I, double m,
    Eigen::MatrixXd& A, Eigen::MatrixXd& Bp, Eigen::MatrixXd&Bf, Eigen::VectorXd& c);

template <typename T>
CentroidalStateDeriv<T> SRBDynamics(
    const CentroidalState<T>& state,
    const std::vector<drake::Vector3<T>>& contact_points,
    const std::vector<drake::Vector3<T>>& contact_forces,
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