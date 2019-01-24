#pragma once

//#include <vector>
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree.h"
#include "multibody/rbt_utils.h"

namespace dairlib {
namespace multibody {

struct ContactInfo {
  Eigen::Matrix3Xd xA;
  Eigen::Matrix3Xd xB;
  std::vector<int> idxA;
};

template <typename T>
class TreeContainer {
 public:
  TreeContainer(const RigidBodyTree<double>& tree, ContactInfo contact_info);

  drake::MatrixX<T> CalcContactJacobian(drake::VectorX<T> q);
  drake::VectorX<T> CalcTimeDerivatives(drake::VectorX<T> x,
                                        drake::VectorX<T> u,
                                        drake::VectorX<T> lambda);
  drake::VectorX<T> CalcMVDot(drake::VectorX<T> x, drake::VectorX<T> u,
                              drake::VectorX<T> lambda);

  ContactInfo get_contact_info();
  int get_num_contacts();
  void set_contact_info(ContactInfo info);

 private:
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  int num_contacts_;
};

}  // namespace multibody
}  // namespace dairlib
