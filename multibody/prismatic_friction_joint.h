#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"

namespace dairlib {
namespace multibody {

/// This Joint extends PrismaticJoint by adding in stiction to the translation
/// axis.
template <typename T>
class PrismaticFrictionJoint final : public PrismaticJoint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticFrictionJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  /// Constructor to create a prismatic joint with stiction between two bodies.
  /// The first seven arguments to this constructor are those of the
  /// PrismaticJoint class constructor. See the PrismaticJoint class's
  /// documentation for details.  The additional input argument is:
  /// @param[in] stiction
  ///   Stiction force limit, in N, for the joint. When the magnitude of the
  ///   applied force is below this limit, the joint resists motion (i.e.
  ///   remains at rest). When the applied force exceeds this limit, the joint
  ///   applies the stiction force opposing the direction of motion.
  /// @throws std::exception if stiction is negative.
  PrismaticFrictionJoint(
      const std::string& name, const Frame<T>& frame_on_parent,
      const Frame<T>& frame_on_child, const Vector3<double>& axis,
      double pos_lower_limit = -std::numeric_limits<double>::infinity(),
      double pos_upper_limit = std::numeric_limits<double>::infinity(),
      double damping = 0, double stiction = 0);

  ~PrismaticFrictionJoint() override;

 protected:
 private:
  // Make PrismaticFrictionJoint templated on every other scalar type a friend
  // of PrismaticFrictionJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can
  // access private members of PrismaticFrictionJoint<T>.
  template <typename>
  friend class PrismaticFrictionJoint;

  // Joint's stiction limit.
  double stiction_;
};

template <typename T>
const char PrismaticFrictionJoint<T>::kTypeName[] = "prismatic_friction";

}  // namespace multibody
}  // namespace dairlib

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticFrictionJoint);
