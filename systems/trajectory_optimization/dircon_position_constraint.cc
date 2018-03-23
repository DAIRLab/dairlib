#include "dircon_position_constraint.h"

namespace drake{

template <typename T>
DirconPositionConstraint<T>::DirconPositionConstraint(const RigidBodyTree<double>& tree, int bodyIdx, Vector3d pt, bool isXZ)
  : DirconKinematicConstraint<T>(tree,isXZ ? 2 : 3) {
  bodyIdx_ = bodyIdx;
  pt_ = pt;
  isXZ_ = isXZ;

  TXZ_ << 1,0,0,
          0,0,1;
}

template <typename T>
DirconPositionConstraint<T>::~DirconPositionConstraint() {
}

template <typename T>
void DirconPositionConstraint<T>::updateConstraint(KinematicsCache<T>& cache) {
  auto pts = this->tree_->transformPoints(cache,pt_,bodyIdx_,0);
  
  //TODO: implement some caching here
  auto v = cache.getV();
  if (isXZ_) {
    this->c_ = TXZ_*pts;
    this->J_ = TXZ_*this->tree_->transformPointsJacobian(cache, pt_.template cast<T>(),bodyIdx_,0, true);
    this->cdot_ = this->J_*v;
    this->Jdotv_ = TXZ_*this->tree_->transformPointsJacobianDotTimesV(cache, pt_.template cast<T>(),bodyIdx_,0);
  } else {
    this->c_ = pts;
    this->J_ = this->tree_->transformPointsJacobian(cache, pt_.template cast<T>(),bodyIdx_,0, true);
    this->cdot_ = this->J_*v;
    this->Jdotv_ = this->tree_->transformPointsJacobianDotTimesV(cache, pt_.template cast<T>(),bodyIdx_,0);
  }
}

// Explicitly instantiates on the most common scalar types.
template class DirconPositionConstraint<double>;
template class DirconPositionConstraint<AutoDiffXd>;
}