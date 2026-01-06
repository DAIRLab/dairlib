#include "multibody/prismatic_friction_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace dairlib {
namespace multibody {

template <typename T>
PrismaticFrictionJoint<T>::PrismaticFrictionJoint(
    const std::string& name, const Frame<T>& frame_on_parent,
    const Frame<T>& frame_on_child, const Vector3<double>& axis,
    double pos_lower_limit, double pos_upper_limit, double damping,
    double stiction)
    : PrismaticJoint<T>(
        name, frame_on_parent, frame_on_child, axis, pos_lower_limit,
        pos_upper_limit, damping) {
  if (stiction < 0) {
    throw std::logic_error("Prismatic joint stiction must be nonnegative.");
  }
  stiction_ = VectorX<double>::Constant(1, stiction);
}

template <typename T>
PrismaticFrictionJoint<T>::~PrismaticFrictionJoint() = default;

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> PrismaticFrictionJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<PrismaticJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->translation_axis(), this->position_lower_limit(),
      this->position_upper_limit(), this->default_damping());
  joint_clone->set_velocity_limits(this->velocity_lower_limits(),
                                   this->velocity_upper_limits());
  joint_clone->set_acceleration_limits(this->acceleration_lower_limits(),
                                       this->acceleration_upper_limits());
  joint_clone->set_default_positions(this->default_positions());

  return joint_clone;
}

template <typename T>
std::unique_ptr<Joint<double>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticJoint);
