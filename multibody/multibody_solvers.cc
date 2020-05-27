#include "multibody/multibody_solvers.h"

namespace dairlib {
namespace multibody {

using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::VectorX;
using Eigen::VectorXd;

using solvers::NonlinearConstraint;

template <typename T>
ContactConstraint<T>::ContactConstraint(const MultibodyPlant<T>& plant,
    const ContactInfo<T>& contact_info, std::shared_ptr<Context<T>> context,
    const std::string& description)
    : NonlinearConstraint<T>(contact_info.num_contacts, plant.num_positions(),
                 VectorXd::Zero(contact_info.num_contacts),
                 VectorXd::Zero(contact_info.num_contacts), description),
      plant_(plant), 
      context_(context) {
  // ContactToolkit pointer using the ContactInfo object.
  contact_toolkit_ =
      std::make_unique<ContactToolkit<T>>(plant, contact_info);
}

template <typename T>
ContactConstraint<T>::ContactConstraint(const MultibodyPlant<T>& plant,
    const ContactInfo<T>& contact_info, const std::string& description) 
    : ContactConstraint<T>(plant, contact_info,
      std::shared_ptr<Context<T>>(plant_.CreateDefaultContext().release()),
      description) {}


template <typename T>
void ContactConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& q, VectorX<T>* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q.size() == plant_.num_positions());

  plant_.SetPositions(context_.get(), q);

  *y = contact_toolkit_->CalcDistanceToGround(*context_);
}


template <typename T>
DistanceConstraint<T>::DistanceConstraint(const MultibodyPlant<T>& plant,
    const Frame<T>* frameA, const Eigen::Vector3d& ptA,
    const Frame<T>* frameB, const Eigen::Vector3d& ptB,
    double distance,
    std::shared_ptr<Context<T>> context, const std::string& description)
    : NonlinearConstraint<T>(1, plant.num_positions(),
                 VectorXd::Zero(1), VectorXd::Zero(1), description),
      plant_(plant), 
      frameA_(frameA),
      ptA_(ptA),
      frameB_(frameB),
      ptB_(ptB),
      distance_(distance),
      context_(context) {}

template <typename T>
DistanceConstraint<T>::DistanceConstraint(const MultibodyPlant<T>& plant,
    const Frame<T>* frameA, const Eigen::Vector3d& ptA,
    const Frame<T>* frameB, const Eigen::Vector3d& ptB,
    double distance,
    const std::string& description) 
    : DistanceConstraint<T>(plant, frameA, ptA, frameB, ptB, distance,
      std::shared_ptr<Context<T>>(plant_.CreateDefaultContext().release()),
      description) {}


template <typename T>
void DistanceConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const VectorX<T>>& q, VectorX<T>* y) const {
  // Verifying the size of the input vector
  DRAKE_DEMAND(q.size() == plant_.num_positions());

  plant_.SetPositions(context_.get(), q);

  // Calculate the position of ptA in frameB
  VectorX<T> ptA_B(3);
  plant_.CalcPointsPositions(*context_, *frameA_,
        ptA_.template cast<T>(), *frameB_, &ptA_B);   

  (*y)(0) = (ptB_ - ptA_B).norm() - distance_;
}

}  // namespace multibody
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::ContactConstraint)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::multibody::DistanceConstraint)
