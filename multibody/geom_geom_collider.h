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
  ///
  /// @param plant
  /// @param geometry_id_A
  /// @param geometry_id_B
  GeomGeomCollider(
      const drake::multibody::MultibodyPlant<T>& plant,
      const drake::SortedPair<drake::geometry::GeometryId> geometry_pair);

  /// Calculates the distance and contact frame Jacobian.
  /// Jacobian is ordered [J_n; J_t], and has shape 3 x (nq or nv), depending
  /// on the choice of JacobianWrtVariable.
  /// @param context The context for the MultibodyPlant
  /// @return A pair with <distance as a scalar, J>
  std::pair<T, drake::MatrixX<T>> Eval(
      const drake::systems::Context<T>& context,
      drake::multibody::JacobianWrtVariable wrt =
          drake::multibody::JacobianWrtVariable::kV);


  /// Calculates the distance and contact frame Jacobian.
  /// Jacobian is ordered [J_n; J_t], and has shape
  ////   (2*num_friction_directions + 1) x (nq or nv), depending
  /// on the choice of JacobianWrtVariable.
  ///
  /// Specifies the number of friction directions, used to
  /// construct a polytope representation of friction with
  /// (2 * num_friction_directions) faces.
  /// Setting this to 0 is acceptable for frictionless contact
  /// num_friction_directions != 1, as this would not be
  /// well-defined in 3D.
  ///
  /// @param context The context for the MultibodyPlant
  /// @param num_friction_directions
  /// @return A pair with <distance as a scalar, J>
  std::pair<T, drake::MatrixX<T>> EvalPolytope(
      const drake::systems::Context<T>& context,
      int num_friction_directions,
      drake::multibody::JacobianWrtVariable wrt =
          drake::multibody::JacobianWrtVariable::kV);


  /// Calculates the distance and contact frame Jacobian for a 2D planar problem
  /// Jacobian is ordered [J_n; +J_t; -J_t], and has shape 3 x (nq).
  /// J_t refers to the (contact_normal x planar_normal) direction
  /// @param context The context for the MultibodyPlant
  /// @param planar_normal The normal vector to the planar system
  /// @return A pair with <distance as a scalar, J>
  std::pair<T, drake::MatrixX<T>> EvalPlanar(
      const drake::systems::Context<T>& context,
      const Eigen::Vector3d& planar_normal,
      drake::multibody::JacobianWrtVariable wrt =
          drake::multibody::JacobianWrtVariable::kV);

 private:
  std::pair<T, drake::MatrixX<T>> DoEval(
      const drake::systems::Context<T>& context,
      Eigen::Matrix<double, Eigen::Dynamic, 3> force_basis,
      drake::multibody::JacobianWrtVariable wrt, bool planar = false);

  const drake::multibody::MultibodyPlant<T>& plant_;
  const drake::geometry::GeometryId geometry_id_A_;
  const drake::geometry::GeometryId geometry_id_B_;
};

}  // namespace multibody
}  // namespace dairlib

extern template class dairlib::multibody::GeomGeomCollider<double>;
