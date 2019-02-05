#include "dircon_contact_data.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix;
using Eigen::Matrix3Xd;
using Eigen::Matrix3d;


namespace dairlib {
template <typename T>
DirconContactData<T>::DirconContactData(RigidBodyTree<double>& tree, std::vector<int>& contact_indices,
                      double mu, bool isXZ)
  : DirconKinematicData<T>(tree,(isXZ ? 2 : 3)*contact_indices.size()) {
  DRAKE_THROW_UNLESS(contact_indices.size() > 0);

  mu_ = mu;
  isXZ_ = isXZ;
  TXZ_ << 1,0,0,
          0,0,1;
  contact_indices_ = contact_indices;

  // call collision detect to count contacts
  VectorXd q0 = VectorXd::Zero(this->tree_->get_num_positions());
  KinematicsCache<double> cache = this->tree_->doKinematics(q0);
  this->tree_->collisionDetect(cache, phi_, normal_, xA_, xB_, idxA_, idxB_);

  int n_contacts = phi_.size();

  if (isXZ_) {
    d_data_ = Matrix3Xd::Zero(3,n_contacts*2);

    MatrixXd A_fric = Matrix<double,2,2>();
    A_fric << mu, 1, mu, -1;
    Vector2d lb_fric = Vector2d::Zero();
    Vector2d ub_fric = VectorXd::Constant(2, std::numeric_limits<double>::infinity());
    auto force_constraint = std::make_shared<drake::solvers::LinearConstraint>(A_fric, lb_fric, ub_fric);

    for (int i = 0; i < n_contacts; i++) {
      Eigen::Map<Matrix3Xd> dmap(d_data_.block(0, 2*i, 3, 2).data(),3,2);
      d_world_.push_back(dmap);

      this->force_constraints_.push_back(force_constraint);
      // auto dynamicConstraint = std::make_shared<DirconDynamicConstraint>(tree, datasetd);
      //std::shared_ptr<drake::solvers::Constraint>;
    }
  } else {
    d_data_ = Matrix3Xd::Zero(3,n_contacts*3);

    Matrix3d A_fric;
    A_fric << mu, 0, 0, 0, 1, 0, 0, 0, 1;
    Vector3d b_fric = Vector3d::Zero();
    auto force_constraint = std::make_shared<drake::solvers::LorentzConeConstraint>(A_fric, b_fric);

    for (int i = 0; i < n_contacts; i++) {
      Eigen::Map<Matrix3Xd> dmap(d_data_.block(0, 3*i, 3, 3).data(),3,3);
      d_world_.push_back(dmap);

      this->force_constraints_.push_back(force_constraint);
    }

  }
}

template <typename T>
DirconContactData<T>::~DirconContactData() {
}

template <typename T>
void DirconContactData<T>::updateConstraint(KinematicsCache<T>& cache) {
  VectorXd q_double = drake::math::DiscardGradient(cache.getQ());
  KinematicsCache<double> cache_double = this->tree_->doKinematics(q_double);

   this->tree_->collisionDetect(cache_double, phi_, normal_, xA_, xB_, idxA_, idxB_);
  int n_contacts = phi_.rows();
  VectorXd n;
  MatrixXd d, basis;
  int num_rows;
  if (isXZ_) {
    num_rows = 2;
    n = Eigen::Vector3d();
    d = Eigen::Vector3d();
    basis = Eigen::Matrix<double,2,3>();
  } else {
    num_rows = 3;
    basis = Eigen::Matrix<double,3,3>();

    const Eigen::Map<Eigen::Matrix3Xd> n_world(normal_.data(),3,n_contacts);
    this->tree_->surfaceTangents(n_world, d_world_);
  }

  //TODO: implement some caching here, check cache.getV and cache.getQ before recomputing
  for (int i=0; i < contact_indices_.size(); i++) {
    int j = contact_indices_[i];
    if (isXZ_) {
      //Normalize x-z component of normal_
      double normal_length = sqrt(normal_(0,j)*normal_(0,j) + normal_(2,j)*normal_(2,j));
      n << normal_(0,j)/normal_length, 0, normal_(2,j)/normal_length;
      d << -n(2), 0, n(0);
      basis << n.transpose(), d.transpose();
    } else {
      basis << normal_.col(j).transpose(), d_world_[j].transpose();
      // d = this->tree_->surfaceTangentsSingle(n, d); //This would be the simplest call, but it's not publically available.
    }

    const Eigen::Vector3d xA_colj = Eigen::Vector3d(xA_.col(j));
    const Eigen::Vector3d xB_colj = Eigen::Vector3d(xB_.col(j));

    auto transform = basis.template cast<T>();

    this->c_.segment(num_rows*i, num_rows) = transform*(this->tree_->transformPoints(cache,xA_colj,idxA_[j],0) -
                                                        this->tree_->transformPoints(cache,xB_colj,idxB_[j],0));
    //m x n
    //todo: confirm sign here
    //todo: this is going to allow everything to slide around---ned to localize against a initial condition. look @ my matlab code
    this->J_.block(num_rows*i,0,num_rows,this->tree_->get_num_positions()) =
      transform*(this->tree_->transformPointsJacobian(cache, xA_colj,idxA_[j],0, true) -
                 this->tree_->transformPointsJacobian(cache, xB_colj,idxB_[j],0, true));
    this->Jdotv_.segment(num_rows*i, num_rows) =
      transform*(this->tree_->transformPointsJacobianDotTimesV(cache, xA_colj,idxA_[j],0) -
                 this->tree_->transformPointsJacobianDotTimesV(cache, xB_colj,idxB_[j],0));
  }
  this->cdot_ = this->J_*cache.getV();
}

// Explicitly instantiates on the most common scalar types.
template class DirconContactData<double>;
template class DirconContactData<AutoDiffXd>;
}