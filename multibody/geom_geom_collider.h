#pragma once

#include "drake/common/sorted_pair.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace multibody {

/// A class for computing collider properties between two geometries
/// Specificaly, used to calculate the signed distance and contact Jacobians
template <typename T>
class GeomGeomCollider {
 public:
  /// Default constructor
  /// Specifies the MultibodyPlant object, as well as the two geometries
  /// Additionally specifies the number of friction directions, used to
  /// construct a polytope representation of friction with
  /// (2 * num_friction_dirctions) faces.
  /// Setting this to 0 is acceptable for frictionless contact
  /// With this constructor, it cannot be set to "1", as this would not be
  /// well-defined in 3D. See alternate constructor below.
  ///
  /// @param plant
  /// @param geometry_id_A
  /// @param geometry_id_B
  /// @param num_friction_directions
  GeomGeomCollider(
      const drake::multibody::MultibodyPlant<T>& plant,
      const drake::SortedPair<drake::geometry::GeometryId> geometry_pair,
      const int num_friction_directions);

  /// This is an alternate constructor for planar systems
  /// Sets num_friction_directions_ = 1
  /// The planar system is defined by a vector, in the world frame,
  /// which is normal to the plane of interest.
  ///
  /// @param plant
  /// @param geometry_id_A
  /// @param geometry_id_B
  /// @param planar_normal
  GeomGeomCollider(
      const drake::multibody::MultibodyPlant<T>& plant,
      const drake::SortedPair<drake::geometry::GeometryId> geometry_pair,
      const Eigen::Vector3d planar_normal);

  /// Calculates the distance and contact Jacobian, set via pointer
  /// Jacobian is ordered [J_n; J_t], and has shape
  /// (1 + 2 * num_friction_directions_) x (nq)
  /// @param context
  /// @param J
  /// @return the distance as a scalar
  std::pair<T, drake::MatrixX<T>> Eval(
      const drake::systems::Context<T>& context);

 private:
  const drake::multibody::MultibodyPlant<T>& plant_;
  const drake::geometry::GeometryId geometry_id_A_;
  const drake::geometry::GeometryId geometry_id_B_;
  const int num_friction_directions_;
  const Eigen::Vector3d planar_normal_;
};

}  // namespace multibody
}  // namespace dairlib

extern template class dairlib::multibody::GeomGeomCollider<double>;
