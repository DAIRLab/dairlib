#pragma once

//#include <vector>
#include "drake/common/default_scalars.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace multibody {

/*
 * ContactInfo structure that holds all the required contact information.
 * xA: Positions on Body A (The robot) expressed in the frame of the robot.
 * xB: Positions on Body B (Assumed to be the ground) expressed in the ground
 * frame (world frame).
 * idxA: Body indices of the corresponding positions in the RigidBodyTree.
 * This structure may be expanded in the future to incorporate more information.
 * Eg: for general body-body contact.
 */
template <typename T>
struct ContactInfo {
  Eigen::Matrix3Xd xA;
  Eigen::Matrix3Xd xB;
  std::vector<const drake::multibody::Frame<T>*> frameA;
};

/*
 * ContactToolkit class for all contact related computations that are tree and
 * contact independent. The model and contact information are specified while
 * constructing an object of the class.
 * None of the functions would run CollisionDetect as all the required contact
 * information is passed through the ContactInfo object.
 * The class is templated so that the AutoDiff variant can be used during
 * optimization.
 */
template <typename T>
class ContactToolkit {
 public:

  // Disabling copy construction and assignment
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactToolkit)

  ContactToolkit(const drake::multibody::MultibodyPlant<T>& plant,
                 ContactInfo<T> contact_info);


  /*
   * Function to compute the contact jacobian. The normal and two surface
   * tangent directions are considered for each contact point-plane contact.
   * @param x The state of the system
   */
  drake::MatrixX<T> CalcContactJacobian(
      const drake::systems::Context<T>& context) const;
  /*
   * Function to compute M * VDot given the state, control inputs and constraint
   * forces
   * @param x The state of the system
   * @param u The control input
   * @param lambda The constraint forces (Includes tree position constraint and
   * contact constraint forces)
   * @pre lambda is the joint vector containing the position constraint force
   * vector followed by the contact force vector (Specifically in this order).
   */
  drake::VectorX<T> CalcMVDot(const drake::systems::Context<T>& context,
                              drake::VectorX<T> lambda) const;
  /*
   * Function to compute xdot given the state, control inputs and constraint
   * forces
   * @param x The state of the system
   * @param u The control input
   * @param lambda The constraint forces (Includes tree position constraint and
   * contact constraint forces)
   * @pre lambda is the joint vector containing the position constraint force
   * vector followed by the contact force vector (Specifically in this order).
   */
  drake::VectorX<T> CalcTimeDerivatives(
      const drake::systems::Context<T>& context,
      drake::VectorX<T> lambda) const;

  ContactInfo<T> get_contact_info();
  int get_num_contacts();
  void set_contact_info(ContactInfo<T> info);

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  ContactInfo<T> contact_info_;
  int num_contacts_;
};

}  // namespace multibody
}  // namespace dairlib
