#include "multibody/geom_geom_collider.h"

#include "drake/math/rotation_matrix.h"
#include <iostream>
using drake::EigenPtr;
using drake::MatrixX;
using drake::VectorX;
using drake::geometry::GeometryId;
using drake::geometry::GeometrySet;
using drake::geometry::SignedDistancePair;
using drake::geometry::SignedDistanceToPoint;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Matrix;
using Eigen::Vector3d;

namespace dairlib {
namespace multibody {

template <typename T>
GeomGeomCollider<T>::GeomGeomCollider(
    const drake::multibody::MultibodyPlant<T>& plant,
    const drake::SortedPair<GeometryId> geometry_pair)
    : plant_(plant),
      geometry_id_A_(geometry_pair.first()),
      geometry_id_B_(geometry_pair.second()) {}

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
        "GeomGeomCollider cannot specify 1 friction direction unless "
        "using EvalPlanar.");
  }

  // Build friction basis
  Matrix<double, Eigen::Dynamic, 3> force_basis(2 * num_friction_directions + 1,
                                                3);
  force_basis.row(0) << 1, 0, 0;

  for (int i = 0; i < num_friction_directions; i++) {
    double theta = (M_PI * i) / num_friction_directions;
    force_basis.row(2 * i + 1) = Vector3d(0, cos(theta), sin(theta));
    force_basis.row(2 * i + 2) = -force_basis.row(2 * i + 1);
  }
  return DoEval(context, force_basis, wrt);
}

template <typename T>
std::pair<T, MatrixX<T>> GeomGeomCollider<T>::EvalPlanar(
    const Context<T>& context, const Vector3d& planar_normal,
    JacobianWrtVariable wrt) {
  return DoEval(context, planar_normal.transpose(), wrt, true);
}

template <typename T>
std::pair<VectorX<double>, VectorX<double>>
GeomGeomCollider<T>::CalcWitnessPoints(const Context<double>& context) {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);
  const auto& inspector = query_object.inspector();

  // Need to set the following:
  Vector3d p_ACa, p_BCb;

  auto [is_sphere_and_mesh, is_A_mesh] = IsSphereAndMesh(context);

  if (is_sphere_and_mesh) {
    Vector3d throwaway_nhat_BA_W;
    T throwaway_distance;
    std::tie(p_ACa, p_BCb, throwaway_nhat_BA_W, throwaway_distance) =
      DoSphereMeshCollision(context, is_A_mesh);
  }

  else {
    const SignedDistancePair<T> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(geometry_id_A_,
                                                          geometry_id_B_);
    p_ACa = inspector.GetPoseInFrame(geometry_id_A_).template cast<T>() *
            signed_distance_pair.p_ACa;
    p_BCb = inspector.GetPoseInFrame(geometry_id_B_).template cast<T>() *
            signed_distance_pair.p_BCb;
  }

  const auto frame_A_id = inspector.GetFrameId(geometry_id_A_);
  const auto frame_B_id = inspector.GetFrameId(geometry_id_B_);
  const auto& frameA = plant_.GetBodyFromFrameId(frame_A_id)->body_frame();
  const auto& frameB = plant_.GetBodyFromFrameId(frame_B_id)->body_frame();

  Vector3d p_WCa = Vector3d::Zero();
  Vector3d p_WCb = Vector3d::Zero();
  plant_.CalcPointsPositions(context, frameA, p_ACa, plant_.world_frame(),
                             &p_WCa);
  plant_.CalcPointsPositions(context, frameB, p_BCb, plant_.world_frame(),
                             &p_WCb);
  return std::pair<VectorX<double>, VectorX<double>>(p_WCa, p_WCb);
}

template <typename T>
std::pair<VectorX<double>, Eigen::Matrix<double, Eigen::Dynamic, 3>>
GeomGeomCollider<T>::CalcWitnessPointsAndForceBasisInWorldFrame(
    const Context<double>& context, bool planar) {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);
  const auto& inspector = query_object.inspector();

  Vector3d p_ACa;

  const SignedDistancePair<T> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(geometry_id_A_,
                                                          geometry_id_B_);
  p_ACa = inspector.GetPoseInFrame(geometry_id_A_).template cast<T>() *
          signed_distance_pair.p_ACa;
  Vector3d nhat_BA_W = signed_distance_pair.nhat_BA_W;

  const auto frame_A_id = inspector.GetFrameId(geometry_id_A_);
  const auto& frameA = plant_.GetBodyFromFrameId(frame_A_id)->body_frame();

  Vector3d p_WCa = Vector3d::Zero();
  plant_.CalcPointsPositions(context, frameA, p_ACa, plant_.world_frame(),
                             &p_WCa);

  // Build friction basis (1 normal, 4 tangents)
  int num_friction_directions = 2;
  Matrix<double, Eigen::Dynamic, 3> force_basis(2 * num_friction_directions + 1,
                                                3);
  if (!planar) {
    force_basis.row(0) << 1, 0, 0;

    for (int i = 0; i < num_friction_directions; i++) {
      double theta = (M_PI * i) / num_friction_directions;
      force_basis.row(2 * i + 1) = Vector3d(0, cos(theta), sin(theta));
      force_basis.row(2 * i + 2) = -force_basis.row(2 * i + 1);
    }

    auto R_WC = drake::math::RotationMatrix<T>::MakeFromOneVector(nhat_BA_W, 0);
    force_basis = force_basis * R_WC.matrix().transpose();
  } else {
    Vector3d planar_normal = force_basis.row(0);
    force_basis = Eigen::MatrixXd::Zero(3, 3);
    force_basis.resize(3, 3);
    // First row is the contact normal, projected to the plane
    force_basis.row(0) =
        nhat_BA_W - planar_normal * planar_normal.dot(nhat_BA_W);
    force_basis.row(0).normalize();

    // Second row is the cross product between contact normal and planar normal
    force_basis.row(1) = nhat_BA_W.cross(planar_normal);
    force_basis.row(1).normalize();
    force_basis.row(2) = -force_basis.row(1);
  }

  return std::pair<VectorX<double>, Eigen::Matrix<double, Eigen::Dynamic, 3>>(
      p_WCa, force_basis);
}


template <typename T>
std::pair<T, MatrixX<T>> GeomGeomCollider<T>::DoEval(
    const Context<T>& context, Matrix<double, Eigen::Dynamic, 3> force_basis,
    JacobianWrtVariable wrt, bool planar) {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);
  const auto& inspector = query_object.inspector();

  // Need to set the following:
  Vector3d p_ACa, p_BCb, nhat_BA_W;
  T distance;

  auto [is_sphere_and_mesh, is_A_mesh] = IsSphereAndMesh(context);

  if (is_sphere_and_mesh) {
    std::tie(p_ACa, p_BCb, nhat_BA_W, distance) = DoSphereMeshCollision(
      context, is_A_mesh);
  }
  else {
    const SignedDistancePair<T> signed_distance_pair =
      query_object.ComputeSignedDistancePairClosestPoints(geometry_id_A_,
                                                          geometry_id_B_);
    p_ACa = inspector.GetPoseInFrame(geometry_id_A_).template cast<T>() *
            signed_distance_pair.p_ACa;
    p_BCb = inspector.GetPoseInFrame(geometry_id_B_).template cast<T>() *
            signed_distance_pair.p_BCb;

    nhat_BA_W = signed_distance_pair.nhat_BA_W;
    distance = signed_distance_pair.distance;
  }


  if (nhat_BA_W.array().isNaN().any()) {
    GeometryId left_wall_id = plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("left_wall"))[0];
    GeometryId right_wall_id = plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("right_wall"))[0];
    GeometryId front_wall_id = plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("front_wall"))[0];
    GeometryId back_wall_id = plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("back_wall"))[0];


    if (geometry_id_A_ == left_wall_id || geometry_id_B_ == left_wall_id) {
      std::cout << "set nhat_BA_W for left_wall" << std::endl;
      nhat_BA_W = {0, -1, 0};
    } else if (geometry_id_A_ == right_wall_id || geometry_id_B_ == right_wall_id) {
      std::cout << "set nhat_BA_W for right_wall" << std::endl;
      nhat_BA_W = {0, 1, 0};
    } else if (geometry_id_A_ == front_wall_id || geometry_id_B_ == front_wall_id) {
      std::cout << "set nhat_BA_W for front_wall" << std::endl;
      nhat_BA_W = {-1, 0, 0};
    } else if (geometry_id_A_ == back_wall_id || geometry_id_B_ == back_wall_id) {
      std::cout << "set nhat_BA_W for back_wall" << std::endl;
      nhat_BA_W = {1, 0, 0};
    } else {
      throw std::runtime_error("GeomGeomCollider: nhat_BA_W is NaN");
    }

  }


  int n_cols = (wrt == JacobianWrtVariable::kV) ? plant_.num_velocities()
                                                : plant_.num_positions();
  Matrix<double, 3, Eigen::Dynamic> Jv_WCa(3, n_cols);
  Matrix<double, 3, Eigen::Dynamic> Jv_WCb(3, n_cols);

  const auto frame_A_id = inspector.GetFrameId(geometry_id_A_);
  const auto frame_B_id = inspector.GetFrameId(geometry_id_B_);
  const auto& frameA = plant_.GetBodyFromFrameId(frame_A_id)->body_frame();
  const auto& frameB = plant_.GetBodyFromFrameId(frame_B_id)->body_frame();

  plant_.CalcJacobianTranslationalVelocity(context, wrt, frameA, p_ACa,
                                           plant_.world_frame(),
                                           plant_.world_frame(), &Jv_WCa);
  plant_.CalcJacobianTranslationalVelocity(context, wrt, frameB, p_BCb,
                                           plant_.world_frame(),
                                           plant_.world_frame(), &Jv_WCb);

  auto R_WC = drake::math::RotationMatrix<T>::MakeFromOneVector(nhat_BA_W, 0);

  // if this is a planar problem, then the basis has one row and encodes
  // the planar normal direction.
  // These calculations cannot easily be moved to the EvalPlanar() method,
  // since they depend so heavily on the contact normal.
  // thus the somewhat awkward calculations here.
  if (planar) {
    Vector3d planar_normal = force_basis.row(0);
    force_basis = Eigen::MatrixXd::Zero(3, 3);
    force_basis.resize(3, 3);
    // First row is the contact normal, projected to the plane
    force_basis.row(0) = nhat_BA_W - planar_normal*planar_normal.dot(nhat_BA_W);
    force_basis.row(0).normalize();

    // Second row is the cross product between contact normal and planar normal
    force_basis.row(1) = nhat_BA_W.cross(planar_normal);
    force_basis.row(1).normalize();
    force_basis.row(2) = -force_basis.row(1);
    R_WC = drake::math::RotationMatrix<T>::Identity();
  }
  // Standard case
  auto J = force_basis * R_WC.matrix().transpose() * (Jv_WCa - Jv_WCb);
  return std::pair<T, MatrixX<T>>(distance, J);
}

template <typename T>
std::pair<bool, bool> GeomGeomCollider<T>::IsSphereAndMesh(
    const drake::systems::Context<T>& context) const {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);
  const auto& inspector = query_object.inspector();

  // Check if the pair is a sphere and mesh -- in this case, collisions can be
  // evaluated w.r.t. the non-convex mesh via custom implementation.
  const auto type_A = inspector.GetShape(geometry_id_A_).type_name();
  const auto type_B = inspector.GetShape(geometry_id_B_).type_name();

  bool is_A_sphere = (type_A == "Sphere");
  bool is_B_sphere = (type_B == "Sphere");
  bool is_A_mesh = (type_A == "Mesh");
  bool is_B_mesh = (type_B == "Mesh");

  bool is_sphere_and_mesh = (is_A_sphere && is_B_mesh) ||
                            (is_B_sphere && is_A_mesh);
  return std::make_pair(is_sphere_and_mesh, is_A_mesh);
}

template <typename T>
std::tuple<Vector3d, Vector3d, Vector3d, T>
  GeomGeomCollider<T>::DoSphereMeshCollision(
    const drake::systems::Context<T>& context, const bool& is_A_mesh) const {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);
  const auto& inspector = query_object.inspector();

  GeometryId geometry_id_mesh = is_A_mesh ? geometry_id_A_ : geometry_id_B_;
  GeometryId geometry_id_sphere = is_A_mesh ? geometry_id_B_ : geometry_id_A_;

  const auto* sphere = dynamic_cast<const drake::geometry::Sphere*>(
    &inspector.GetShape(geometry_id_sphere));
  T sphere_radius = sphere->radius();

  // Get the pose of the sphere in its own frame.
  auto sphere_pose_in_sphere = inspector.GetPoseInFrame(
    geometry_id_sphere).template cast<T>();
  // Convert to world frame.
  auto sphere_frame_id = inspector.GetFrameId(geometry_id_sphere);
  auto X_WS = plant_.EvalBodyPoseInWorld(
    context, *plant_.GetBodyFromFrameId(sphere_frame_id));
  auto sphere_pose = X_WS * sphere_pose_in_sphere;

  Vector3d sphere_center = sphere_pose.translation();
  GeometrySet geometry_set_with_mesh;
  geometry_set_with_mesh.Add(geometry_id_mesh);
  const SignedDistanceToPoint<T> signed_distance_to_point =
    query_object.ComputeSignedDistanceGeometryToPoint(
      sphere_center, geometry_set_with_mesh)[0];
  // Set the values.
  Vector3d p_ACa, p_BCb;
  T distance = signed_distance_to_point.distance - sphere_radius;
  Vector3d nhat_BA_W = signed_distance_to_point.grad_W.normalized();
  //std::cout << nhat_BA_W.transpose() << std::endl;
  if (is_A_mesh) {
    nhat_BA_W = -nhat_BA_W;

    p_ACa = inspector.GetPoseInFrame(geometry_id_A_).template cast<T>() *
          signed_distance_to_point.p_GN;
    p_BCb = sphere_pose_in_sphere.template cast<T>() *
      (-1 * sphere_radius * nhat_BA_W);
  }
  else {
    p_BCb = inspector.GetPoseInFrame(geometry_id_B_).template cast<T>() *
          signed_distance_to_point.p_GN;
    p_ACa = sphere_pose_in_sphere.template cast<T>() *
      (-1 * sphere_radius * nhat_BA_W);
  }

  return std::make_tuple(p_ACa, p_BCb, nhat_BA_W, distance);
}

}  // namespace multibody
}  // namespace dairlib

template class dairlib::multibody::GeomGeomCollider<double>;