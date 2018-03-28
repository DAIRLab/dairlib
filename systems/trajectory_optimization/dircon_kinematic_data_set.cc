#include "dircon_kinematic_data_set.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

#include <chrono>

namespace drake{
template <typename T>
DirconKinematicDataSet<T>::DirconKinematicDataSet(const RigidBodyTree<double>& tree, std::vector<DirconKinematicData<T>*>* constraints) :
  DirconKinematicDataSet(tree,constraints, tree.get_num_positions(), tree.get_num_velocities()) {}

template <typename T>
DirconKinematicDataSet<T>::DirconKinematicDataSet(const RigidBodyTree<double>& tree, std::vector<DirconKinematicData<T>*>* constraints, int num_positions, int num_velocities) {
  tree_ = &tree;

  constraints_ = constraints;
  num_positions_ = num_positions;
  num_velocities_ = num_velocities;
  // Initialize matrices
  num_constraints_ = 0;
  for (int i=0; i < constraints_->size(); i++) {
    num_constraints_ += (*constraints_)[i]->getLength();
  }
  c_ = VectorX<T>(num_constraints_);
  cdot_ = VectorX<T>(num_constraints_);
  J_ = MatrixX<T>(num_constraints_,num_positions);
  Jdotv_ = VectorX<T>(num_constraints_);
  cddot_ = VectorX<T>(num_constraints_);
  vdot_ = VectorX<T>(num_velocities_);
  xdot_ = VectorX<T>(num_positions_ + num_velocities_);
}


template <typename T>
void DirconKinematicDataSet<T>::updateData(const VectorX<T>& state, const VectorX<T>& input, const VectorX<T>& forces) {
  const VectorX<T> q = state.head(num_positions_);
  const VectorX<T> v = state.tail(num_velocities_);
  KinematicsCache<T> cache = tree_->doKinematics(q, v, true);

  int index = 0;
  int n;
  for (int i=0; i < constraints_->size(); i++) {
    (*constraints_)[i]->updateConstraint(cache);

    n = (*constraints_)[i]->getLength();
    c_.segment(index, n) = (*constraints_)[i]->getC();
    cdot_.segment(index, n) = (*constraints_)[i]->getCDot();
    J_.block(index, 0, n, num_positions_) = (*constraints_)[i]->getJ();
    Jdotv_.segment(index, n) = (*constraints_)[i]->getJdotv();

    index += n;
  }

  /*
  Eigen::VectorXd q2(4);
  Eigen::VectorXd v2(4);
  Eigen::VectorXd x2(8);
  q2 << 1,2,3,4.1;
  v2 << -1,-3.2,-4,-2.5;
  x2 <<q2,v2;

  VectorX<AutoDiffUpTo73d> xd = VectorX<AutoDiffUpTo73d>::Ones(8);
  VectorX<AutoDiffUpTo73d> qd = xd.segment(0,4);
  VectorX<AutoDiffUpTo73d> vd = xd.segment(4,4);

  auto cache2 = tree_->doKinematics(qd,vd,true);


//  MatrixX<AutoDiffXd> M2(4,4);
  auto start = std::chrono::high_resolution_clock::now();
  for (int i=0; i < 1000; i++)
     auto M2 = tree_->massMatrix(cache2);
  std::cout << "C:" << cache2.areInertiasCached() <<std::endl; 
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "T:" << elapsed.count() <<std::endl;
  */

  const MatrixX<T> M = tree_->massMatrix(cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
  const MatrixX<T> J_transpose = getJ().transpose();

  // right_hand_side is the right hand side of the system's equations:
  // M*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side = -tree_->dynamicsBiasTerm(cache, no_external_wrenches) + tree_->B*input + J_transpose*forces;
  vdot_ = M.llt().solve(right_hand_side);

  cddot_ = Jdotv_ + J_*vdot_;

  xdot_ << tree_->GetVelocityToQDotMapping(cache)*v, vdot_; //assumes v = qdot
}

template <typename T>
int DirconKinematicDataSet<T>::getNumConstraints() {
  return num_constraints_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getC() {
  return c_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getCDot() {
  return cdot_;
}

template <typename T>
MatrixX<T> DirconKinematicDataSet<T>::getJ() {
  return J_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getJdotv() {
  return Jdotv_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getCDDot() {
  return cddot_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getVDot() {
  return vdot_;
}

template <typename T>
VectorX<T> DirconKinematicDataSet<T>::getXDot() {
  return xdot_;
}

template <typename T>
DirconKinematicData<T>* DirconKinematicDataSet<T>::getConstraint(int index) {
  return (*constraints_)[index];
}


// Explicitly instantiates on the most common scalar types.
template class DirconKinematicDataSet<double>;
template class DirconKinematicDataSet<AutoDiffXd>;

}