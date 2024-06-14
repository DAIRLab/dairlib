#include "gravity_compensator.h"

#include "systems/framework/timestamped_vector.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {


GravityCompensationRemover::GravityCompensationRemover(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context)
    : plant_(plant), context_(context){

  num_actuators_ = plant_.num_actuators();
  this->DeclareVectorInputPort("u, t",
                               TimestampedVector<double>(num_actuators_));
  this->DeclareVectorOutputPort("u, t",
                                TimestampedVector<double>(num_actuators_),
                                &GravityCompensationRemover::CancelGravityCompensation);
}

void GravityCompensationRemover::CancelGravityCompensation(const drake::systems::Context<double>& context,
                                                           TimestampedVector<double>* output) const {
  const TimestampedVector<double>* tau =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  VectorXd compensated_tau = VectorXd::Zero(num_actuators_);
  for (int i = 0; i < num_actuators_; i++){
    compensated_tau(i) = tau->GetAtIndex(i) + tau_g(i);
  }

  output->SetDataVector(compensated_tau);
  output->set_timestamp(tau->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib