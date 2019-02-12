#include <limits>
#include <vector>

#include "attic/systems/trajectory_optimization/dircon_position_data.h"

namespace dairlib {

using Eigen::Vector2d;
using Eigen::Vector3d;

template <typename T>
DirconPositionData<T>::DirconPositionData(const RigidBodyTree<double>& tree,
                                          int bodyIdx, Vector3d pt, bool isXZ)
    : DirconKinematicData<T>(tree, isXZ ? 2 : 3) {
  bodyIdx_ = bodyIdx;
  pt_ = pt;
  isXZ_ = isXZ;

  TXZ_ << 1, 0, 0,
          0, 0, 1;
}

template <typename T>
DirconPositionData<T>::~DirconPositionData() {
}

template <typename T>
void DirconPositionData<T>::updateConstraint(const KinematicsCache<T>& cache) {
  auto pts = this->tree_->transformPoints(cache, pt_, bodyIdx_, 0);

  // TODO(mposa): implement some caching here, check cache.getV and cache.getQ
  // before recomputing
  auto v = cache.getV();
  if (isXZ_) {
    this->c_ = TXZ_*pts;
    this->J_ = TXZ_*this->tree_->transformPointsJacobian(cache, pt_, bodyIdx_,
                                                         0, true);
    this->cdot_ = this->J_*v;
    this->Jdotv_ = TXZ_*this->tree_->transformPointsJacobianDotTimesV(cache,
        pt_, bodyIdx_, 0);
  } else {
    this->c_ = pts;
    this->J_ = this->tree_->transformPointsJacobian(cache, pt_, bodyIdx_, 0,
                                                    true);
    this->cdot_ = this->J_*v;
    this->Jdotv_ = this->tree_->transformPointsJacobianDotTimesV(cache, pt_,
                                                                 bodyIdx_, 0);
  }
}

template <typename T>
void DirconPositionData<T>::addFixedNormalFrictionConstraints(Vector3d normal,
                                                              double mu) {
  if (isXZ_) {
    Vector2d normal_xz, d_xz;
    double L = sqrt(normal(0)*normal(0) + normal(2)*normal(2));
    normal_xz << normal(0)/L, normal(2)/L;
    d_xz << -normal_xz(1), normal_xz(0);

    Eigen::Matrix2d A_fric;
    A_fric << (mu*normal_xz + d_xz).transpose(),
              (mu*normal_xz - d_xz).transpose();
    Vector2d lb_fric = Vector2d::Zero();
    Vector2d ub_fric = Vector2d::Constant(
        std::numeric_limits<double>::infinity());

    auto force_constraint = std::make_shared<drake::solvers::LinearConstraint>(
        A_fric, lb_fric, ub_fric);
    this->force_constraints_.push_back(force_constraint);
  } else {
    // Awkward construction here using the tree
    std::vector<Eigen::Map<Eigen::Matrix3Xd>> d_world;
    Eigen::Map<Eigen::Matrix3Xd> n_world(normal.data(), 3, 1);
    this->tree_->surfaceTangents(n_world, d_world);

    Eigen::Matrix3d A_fric;
    A_fric << mu*normal.transpose(), d_world[0].transpose();
    Vector3d b_fric = Vector3d::Zero();
    auto force_constraint =
        std::make_shared<drake::solvers::LorentzConeConstraint>(A_fric, b_fric);
    this->force_constraints_.push_back(force_constraint);
  }
}


// Explicitly instantiates on the most common scalar types.
template class DirconPositionData<double>;
template class DirconPositionData<drake::AutoDiffXd>;
}  // namespace dairlib
