#include "dircon_position_constraint.h"

namespace drake{

template <typename T>
DirconPositionConstraint<T>::DirconPositionConstraint(RigidBodyTree<double>* tree, int bodyIdx, Vector3d pt, bool isXZ) 
  : DirconKinematicConstraint<T>(tree,isXZ ? 2 : 3) {
  bodyIdx_ = bodyIdx;
  pt_ = pt;
  isXZ_ = isXZ;
}

template <typename T>
DirconPositionConstraint<T>::~DirconPositionConstraint() {
}

template <typename T>
void DirconPositionConstraint<T>::updateConstraint(KinematicsCache<T>& cache) {
  auto pts = this->tree_->transformPoints(cache,pt_,bodyIdx_,0);
  
  auto v = cache.getV();
  this->c_ = pts;
  this->J_ = this->tree_->transformPointsJacobian(cache, pt_.template cast<T>(),bodyIdx_,0, true);
  this->cdot_ = this->J_*v;
  this->Jdotv_ = this->tree_->transformPointsJacobianDotTimesV(cache, pt_.template cast<T>(),bodyIdx_,0);
}

// Explicitly instantiates on the most common scalar types.
template class DirconPositionConstraint<double>;
template class DirconPositionConstraint<AutoDiffXd>;
}