#include "dircon_contact_data.h"
#include "drake/math/autodiff.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;


namespace drake{
template <typename T>
DirconContactData<T>::DirconContactData(RigidBodyTree<double>& tree, std::vector<int>& contact_indices,
                      double mu, bool isXZ)
  : DirconKinematicData<T>(tree,(isXZ ? 2 : 3)*contact_indices.size()) {
  mu_ = mu;
  isXZ_ = isXZ;
  TXZ_ << 1,0,0,
          0,0,1;
  contact_indices_ = contact_indices_;

  //Add friction constraints
  if (isXZ_) {

  } else {

  }
}

template <typename T>
DirconContactData<T>::~DirconContactData() {
}

template <typename T>
void DirconContactData<T>::updateConstraint(KinematicsCache<T>& cache) {
  VectorXd q_double = math::DiscardGradient(cache.getQ());
  KinematicsCache<double> cache_double = this->tree_->doKinematics(q_double);

  this->tree_->collisionDetect(cache_double, phi_, normal_, xA_, xB_, idxA_, idxB_);

  //construct contact basis
  MatrixXd basis;
  if (isXZ_) {
  // const Eigen::Map<Eigen::Matrix3Xd> n_world(normal.data(),3,2);
  // std::vector<Eigen::Map<Matrix3Xd>> d_world;
  // Matrix3Xd d1 = Matrix3Xd::Zero(3,2);
  // Matrix3Xd d2 = Matrix3Xd::Zero(3,2);
  // Eigen::Map<Matrix3Xd> d1map(d1.data(),3,2);
  // Eigen::Map<Matrix3Xd> d2map(d2.data(),3,2);
  // d_world.push_back(d1map);
  // d_world.push_back(d2map);
  // tree.get()->surfaceTangents(n_world, d_world);

  // Eigen::Matrix<double,3,2> d;
  // surfaceTangents(normal.col(0),d);


  }


  //TODO: implement some caching here, check cache.getV and cache.getQ before recomputing
  this->c_ = phi_;
  for (int i=0; i < contact_indices_.size(); i++) {
    int j = contact_indices_[i];
    auto pts = basis*(this->tree_->transformPoints(cache,xA_.col(j),idxA_[j],0) -
                      this->tree_->transformPoints(cache,xB_.col(j),idxB_[j],0));

    const Eigen::Vector3d xA_colj = Eigen::Vector3d(xA_.col(j));
    const Eigen::Vector3d xB_colj = Eigen::Vector3d(xB_.col(j));

    //todo need to multiply by J anc C
    // this->J_.row(i) = basis*(this->tree_->transformPointsJacobian(cache, xA_colj.template cast<T>(),idxA_[j],0, true) -
                            // this->tree_->transformPointsJacobian(cache, xB_colj.template cast<T>(),idxB_[j],0, true));
    //this->Jdotv_.row(i) = basis*(this->tree_->transformPointsJacobianDotTimesV(cache, xA_colj.template cast<T>(),idxA_[j],0) -
    //                             this->tree_->transformPointsJacobianDotTimesV(cache, xB_colj.template cast<T>(),idxB_[j],0));
  }
  this->cdot_ = this->J_*cache.getV();
}

// Explicitly instantiates on the most common scalar types.
template class DirconContactData<double>;
template class DirconContactData<AutoDiffXd>;
}