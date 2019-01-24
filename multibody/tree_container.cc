#include "multibody/tree_container.h"

namespace dairlib {
namespace multibody {

template <typename T>
TreeContainer<T>::TreeContainer(const RigidBodyTree<double>& tree,
                                ContactInfo contact_info)
    : tree_(tree), contact_info_(contact_info) {}

template <typename T>
ContactInfo TreeContainer<T>::get_contact_info() {
  return contact_info_;
}

template <typename T>
int TreeContainer<T>::get_num_contacts() {
  return contact_info_.num_contacts;
}

template<typename T>
void TreeContainer<T>::set_contact_info(ContactInfo contact_info) {
  contact_info_ = contact_info;
}

}  // namespace multibody
}  // namespace dairlib
