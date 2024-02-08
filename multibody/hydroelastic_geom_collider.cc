#include "multibody/hydroelastic_geom_collider.h"

#include <iostream>

#include "drake/math/rotation_matrix.h"

using drake::EigenPtr;
using drake::MatrixX;
using drake::VectorX;
using drake::geometry::GeometryId;
using drake::geometry::HydroelasticContactRepresentation;
using drake::geometry::SignedDistancePair;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace multibody {

template <typename T>
HydroelasticGeomCollider<T>::HydroelasticGeomCollider(
    const drake::multibody::MultibodyPlant<T>& plant,
    const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
        contact_geoms)
    : plant_(plant), contact_pairs_(contact_geoms) {}

template <typename T>
std::vector<std::pair<T, MatrixX<T>>> HydroelasticGeomCollider<T>::Eval(
    const Context<T>& context, JacobianWrtVariable wrt) {
  return DoEval(context, Eigen::Matrix3d::Identity(), wrt);
}

template <typename T>
std::vector<std::pair<T, MatrixX<T>>> HydroelasticGeomCollider<T>::EvalPolytope(
    const Context<T>& context, int num_friction_directions,
    JacobianWrtVariable wrt) {
  if (num_friction_directions == 1) {
    throw std::runtime_error(
        "HydroelasticGeomCollider cannot specify 1 friction direction unless "
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
std::vector<std::pair<T, MatrixX<T>>> HydroelasticGeomCollider<T>::EvalPlanar(
    const Context<T>& context, const Vector3d& planar_normal,
    JacobianWrtVariable wrt) {
  return DoEval(context, planar_normal.transpose(), wrt, true);
}

template <typename T>
std::vector<VectorX<double>> HydroelasticGeomCollider<T>::CalcWitnessPoints(
    const Context<double>& context) {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);

  auto geom_ids = contact_pairs_.front();
  auto geometry_id_A_ = geom_ids.first();
  auto geometry_id_B_ = geom_ids.second();

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
  Vector3d p_WCa = Vector3d::Zero();
  Vector3d p_WCb = Vector3d::Zero();
  plant_.CalcPointsPositions(context, frameA, p_ACa, plant_.world_frame(),
                             &p_WCa);
  plant_.CalcPointsPositions(context, frameB, p_BCb, plant_.world_frame(),
                             &p_WCb);
  std::vector<VectorX<double>> witness_points;
  witness_points.push_back(p_WCa);
  return witness_points;
}

template <typename T>
std::vector<std::pair<T, MatrixX<T>>> HydroelasticGeomCollider<T>::DoEval(
    const Context<T>& context, Matrix<double, Eigen::Dynamic, 3> force_basis,
    JacobianWrtVariable wrt, bool planar) {
  const auto& query_port = plant_.get_geometry_query_input_port();
  const auto& query_object =
      query_port.template Eval<drake::geometry::QueryObject<T>>(context);
  const auto& inspector = query_object.inspector();

  const auto& contact_surfaces = query_object.ComputeContactSurfaces(
      HydroelasticContactRepresentation::kPolygon);
//  auto contact_info = drake::multibody::internal::HydroelasticContactInfoAndBodySpatialForces<T>(
//      plant_.num_bodies());
//  plant_.CalcHydroelasticContactForces(context, &contact_info);
  std::vector<std::pair<T, MatrixX<T>>> contact_data;
  std::set<drake::SortedPair<drake::geometry::GeometryId>>
      geom_pairs_with_data;
  for (auto& contact_surface : contact_surfaces) {
    std::cout << "body M name: "
              << plant_
                     .GetBodyFromFrameId(
                         inspector.GetFrameId(contact_surface.id_M()))
                     ->name()
              << std::endl;
    std::cout << "body N name: "
              << plant_
                     .GetBodyFromFrameId(
                         inspector.GetFrameId(contact_surface.id_N()))
                     ->name()
              << std::endl;
    std::cout << "Has M gradient" << contact_surface.HasGradE_M() << std::endl;
    std::cout << "Has N gradient" << contact_surface.HasGradE_N() << std::endl;

    std::cout << "Total area: " << contact_surface.total_area() << std::endl;
    std::cout << "contact_surface.num_faces()" << contact_surface.num_faces()
              << std::endl;
    const SignedDistancePair<T> contact_surface_signed_distance_pair =
        query_object.ComputeSignedDistancePairClosestPoints(
            contact_surface.id_M(), contact_surface.id_N());

    //    const auto frame_B_id = inspector.GetFrameId(geometry_id_B_);
    const auto& frameM =
        plant_.GetBodyFromFrameId(inspector.GetFrameId(contact_surface.id_M()))
            ->body_frame();
    const auto& frameN =
        plant_.GetBodyFromFrameId(inspector.GetFrameId(contact_surface.id_N()))
            ->body_frame();

    const Vector3d& p_ACM =
        inspector.GetPoseInFrame(contact_surface.id_M()).template cast<T>() *
        contact_surface_signed_distance_pair.p_ACa;
    const Vector3d& p_BCN =
        inspector.GetPoseInFrame(contact_surface.id_N()).template cast<T>() *
        contact_surface_signed_distance_pair.p_BCb;

    Vector3d p_WCa = Vector3d::Zero();
    Vector3d p_WCb = Vector3d::Zero();
    plant_.CalcPointsPositions(context, frameM, p_ACM, plant_.world_frame(),
                               &p_WCa);
    plant_.CalcPointsPositions(context, frameN, p_BCN, plant_.world_frame(),
                               &p_WCb);
    int face_with_area = 0;
    VectorXd wrench_basis = VectorXd::Zero(6);

    for (int i = 0; i < contact_surface.num_faces(); ++i) {
      if (contact_surface.area(i) > .0001) {
        std::cout << "face centroid: " << contact_surface.centroid(i)
                  << std::endl;
        if (contact_surface.HasGradE_N()) {
          std::cout << "Grad N" << contact_surface.EvaluateGradE_N_W(i)
                    << std::endl;
        }
        face_with_area += 1;
        VectorXd wrench = VectorXd::Zero(6);
        wrench << contact_surface.face_normal(i),
            (contact_surface.centroid(i) - p_WCb)
                .cross(contact_surface.face_normal(i));
        wrench_basis += contact_surface.area(i) * wrench;
      }
    }
    std::cout << "wrench basis: " << wrench_basis << std::endl;
    std::cout << "faces with non-negligible area: " << face_with_area
              << std::endl;
    int n_cols = (wrt == JacobianWrtVariable::kV) ? plant_.num_velocities()
                                                  : plant_.num_positions();
//    Matrix<double, 3, Eigen::Dynamic> Jv_WCa(3, n_cols);
    Matrix<double, 6, Eigen::Dynamic> Js_V_ABp_E(6, n_cols);
    plant_.CalcJacobianSpatialVelocity(context, wrt, frameM, contact_surface_signed_distance_pair.p_ACa,
                                             plant_.world_frame(),
                                             plant_.world_frame(), &Js_V_ABp_E);
    plant_.CalcJacobianSpatialVelocity(context, wrt, frameM, contact_surface_signed_distance_pair.p_ACa,
                                       plant_.world_frame(),
                                       plant_.world_frame(), &Js_V_ABp_E);
    contact_data.push_back(std::pair<T, MatrixX<T>>(contact_surface_signed_distance_pair.distance, Js_V_ABp_E));
    geom_pairs_with_data.insert(
        drake::SortedPair<drake::geometry::GeometryId>(contact_surface.id_M(),
                                                       contact_surface.id_N()));
  }

  for (auto &contact_pair : contact_pairs_){
    if (geom_pairs_with_data.find(contact_pair) == geom_pairs_with_data.end()){
      auto geometry_id_M = contact_pair.first();
      auto geometry_id_N = contact_pair.second();
      const SignedDistancePair<T> signed_distance_pair =
          query_object.ComputeSignedDistancePairClosestPoints(geometry_id_M,
                                                              geometry_id_N);
      const auto frame_A_id = inspector.GetFrameId(geometry_id_M);
      const auto frame_B_id = inspector.GetFrameId(geometry_id_N);
      const auto& frameA = plant_.GetBodyFromFrameId(frame_A_id)->body_frame();
      const auto& frameB = plant_.GetBodyFromFrameId(frame_B_id)->body_frame();

      const Vector3d& p_ACa =
          inspector.GetPoseInFrame(geometry_id_M).template cast<T>() *
              signed_distance_pair.p_ACa;
      const Vector3d& p_BCb =
          inspector.GetPoseInFrame(geometry_id_N).template cast<T>() *
              signed_distance_pair.p_BCb;

      int n_cols = (wrt == JacobianWrtVariable::kV) ? plant_.num_velocities()
                                                    : plant_.num_positions();
      Matrix<double, 3, Eigen::Dynamic> Jv_WCa(3, n_cols);
      Matrix<double, 3, Eigen::Dynamic> Jv_WCb(3, n_cols);

      plant_.CalcJacobianTranslationalVelocity(context, wrt, frameA, p_ACa,
                                               plant_.world_frame(),
                                               plant_.world_frame(), &Jv_WCa);
      plant_.CalcJacobianTranslationalVelocity(context, wrt, frameB, p_BCb,
                                               plant_.world_frame(),
                                               plant_.world_frame(), &Jv_WCb);

      const auto& R_WC = drake::math::RotationMatrix<T>::MakeFromOneVector(
          signed_distance_pair.nhat_BA_W, 0);

      // Standard case
      auto J = force_basis * R_WC.matrix().transpose() * (Jv_WCa - Jv_WCb);
      contact_data.push_back(std::pair<T, MatrixX<T>>(signed_distance_pair.distance, J));
    }
  }
}

}  // namespace multibody
}  // namespace dairlib

template class dairlib::multibody::HydroelasticGeomCollider<double>;