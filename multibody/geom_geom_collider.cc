#include "multibody/geom_geom_collider.h"
#include "multibody/geom_geom_collider.h"

#include "drake/math/rotation_matrix.h"

using Eigen::Vector3d;
using Eigen::Matrix;
using drake::EigenPtr;
using drake::MatrixX;
using drake::geometry::GeometryId;
using drake::geometry::SignedDistancePair;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib {
namespace multibody {

template <typename T>
GeomGeomCollider<T>::GeomGeomCollider(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::SortedPair<drake::geometry::GeometryId> geometry_pair)
    : plant_(plant),
      geometry_id_A_(geometry_pair.first()),
      geometry_id_B_(geometry_pair.second()) {
}

template <typename T>
std::pair<T, MatrixX<T>> GeomGeomCollider<T>::Eval(const Context<T>& context,
                                                   JacobianWrtVariable wrt) {
  return DoEval(context, Eigen::Matrix3d::Identity(), wrt);
}

template <typename T>
std::pair<T, MatrixX<T>> GeomGeomCollider<T>::EvalPolytope(
    const Context<T>& context, int num_friction_directions,
    JacobianWrtVariable wrt) {

  if (num_friction_directions == 1) {
    throw std::runtime_error(
      "GeomGeomCollider cannot specificy 1 friction direction unless "
          "using EvalPlanar.");
  }

  // Build friction basis
  Matrix<double, Eigen::Dynamic, 3> force_basis(
      2 * num_friction_directions + 1, 3);
  force_basis.row(0) << 1, 0, 0;

  for (int i = 0; i < num_friction_directions; i++) {
    double theta = (M_PI * i) / num_friction_directions;
    force_basis.row(2*i + 1) = Vector3d(0, cos(theta), sin(theta));
    force_basis.row(2*i + 2) = -force_basis.row(2*i + 1);
  }
  return DoEval(context, force_basis, wrt);
}

template <typename T>
std::pair<T, MatrixX<T>> GeomGeomCollider<T>::EvalPlanar(
    const Context<T>& context, const Vector3d& planar_normal,
    JacobianWrtVariable wrt) {
  return DoEval(context, planar_normal.transpose(), wrt, true);
}

  // }
template <typename T>
std::pair<T, MatrixX<T>> GeomGeomCollider<T>::DoEval(
    const Context<T>& context, Matrix<double, Eigen::Dynamic, 3> force_basis,
    JacobianWrtVariable wrt, bool planar) {

  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);

  const SignedDistancePair<T> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(geometry_id_A_,
                                                          geometry_id_B_);

  const auto& inspector = query_object.inspector();
  const auto frame_A_id = inspector.GetFrameId(geometry_id_A_);
  const auto frame_B_id = inspector.GetFrameId(geometry_id_B_);
  const auto& frameA = plant_.GetBodyFromFrameId(frame_A_id)->body_frame();
  const auto& frameB = plant_.GetBodyFromFrameId(frame_B_id)->body_frame();

  const Vector3d& p_ACa =
      inspector.GetPoseInFrame(geometry_id_A_).template cast<T>() *
          signed_distance_pair.p_ACa;
  const Vector3d& p_BCb =
      inspector.GetPoseInFrame(geometry_id_B_).template cast<T>() *
          signed_distance_pair.p_BCb;


  int n_cols = (wrt == JacobianWrtVariable::kV) ? plant_.num_velocities() :
      plant_.num_positions();
  Matrix<double, 3, Eigen::Dynamic> Jv_WCa(3, n_cols);
  Matrix<double, 3, Eigen::Dynamic> Jv_WCb(3, n_cols);

  plant_.CalcJacobianTranslationalVelocity(context, wrt,
                                            frameA, p_ACa, plant_.world_frame(),
                                            plant_.world_frame(), &Jv_WCa);
  plant_.CalcJacobianTranslationalVelocity(context, wrt,
                                            frameB, p_BCb, plant_.world_frame(),
                                            plant_.world_frame(), &Jv_WCb);

  const auto& R_WC =
      drake::math::RotationMatrix<T>::MakeFromOneVector(
          signed_distance_pair.nhat_BA_W, 0);

  // if this is a planar problem, then the basis has one row and encodes
  // the planar normal direction.
  // These calculations cannot easily be moved to the EvalPlanar() method,
  // since they depend so heavily on the contact normal.
  // thus the somewhat awkward calculations here.
  if (planar) {
    Vector3d planar_normal = force_basis.row(0);
    force_basis.resize(3, 3);

    // First row is the contact normal, projected to the plane
    force_basis.row(0) = signed_distance_pair.nhat_BA_W -
        planar_normal * planar_normal.dot(signed_distance_pair.nhat_BA_W);
    force_basis.row(0).normalize();

    // Second row is the cross product between contact normal and planar normal
    force_basis.row(1) = signed_distance_pair.nhat_BA_W.cross(planar_normal);
    force_basis.row(1).normalize();
    force_basis.row(2) = -force_basis.row(1);
  }
    // Standard case
  auto J = force_basis * R_WC.matrix().transpose() * (Jv_WCa - Jv_WCb);
  return std::pair<T, MatrixX<T>>(signed_distance_pair.distance, J);
}

}  // namespace multibody
}  // namespace dairlib


template class dairlib::multibody::GeomGeomCollider<double>;