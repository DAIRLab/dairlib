#include "alip_utils.h"
#include "drake/math/autodiff_gradient.h"


namespace dairlib::systems::controllers::cf_mpfc_utils {

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
using CentroidalState = Eigen::Matrix<T, 18, 1>;

template <typename T>
using CentroidalStateDeriv = Eigen::Matrix<T, 18, 1>;

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
 */
CentroidalState<double> GetCentroidalState(
    const drake::multibody::MultibodyPlant<double> &plant,
    const drake::systems::Context<double> &plant_context,
    const drake::multibody::Frame<double> &floating_base_frame,
    const std::function<Eigen::Matrix3d(
        const drake::multibody::MultibodyPlant<double> &,
        const drake::systems::Context<double> &)> &acom_function,
    const alip_utils::PointOnFramed &stance_foot);

template <typename T>
CentroidalStateDeriv<T> SRBDynamics(
    const CentroidalState<T>& state,
    const std::vector<drake::Vector6<T>>& stacked_contact_locations_and_forces,
    const Eigen::Matrix3d& I, double m);

}