#include "dircon_position_data.h"

namespace drake{

template <typename T>
DirconPositionData<T>::DirconPositionData(RigidBodyTree<double>& tree, int bodyIdx, Vector3d pt, bool isXZ)
  : DirconKinematicData<T>(tree,isXZ ? 2 : 3) {
  bodyIdx_ = bodyIdx;
  pt_ = pt;
  isXZ_ = isXZ;

  TXZ_ << 1,0,0,
          0,0,1;
}

template <typename T>
DirconPositionData<T>::~DirconPositionData() {
}

template <typename T>
void DirconPositionData<T>::updateConstraint(KinematicsCache<T>& cache) {
  auto pts = this->tree_->transformPoints(cache,pt_,bodyIdx_,0);

  //TODO: implement some caching here, check cache.getV and cache.getQ before recomputing
  auto v = cache.getV();
  if (isXZ_) {
    this->c_ = TXZ_*pts;
    this->J_ = TXZ_*this->tree_->transformPointsJacobian(cache, pt_,bodyIdx_,0, true);
    this->cdot_ = this->J_*v;
    this->Jdotv_ = TXZ_*this->tree_->transformPointsJacobianDotTimesV(cache, pt_,bodyIdx_,0);
  } else {
    this->c_ = pts;
    this->J_ = this->tree_->transformPointsJacobian(cache, pt_,bodyIdx_,0, true);
    this->cdot_ = this->J_*v;
    this->Jdotv_ = this->tree_->transformPointsJacobianDotTimesV(cache, pt_,bodyIdx_,0);
  }
}

// Explicitly instantiates on the most common scalar types.
template class DirconPositionData<double>;
template class DirconPositionData<AutoDiffXd>;
}