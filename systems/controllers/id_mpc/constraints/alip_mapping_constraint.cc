#include "alip_mapping_constraint.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::Vector3d;

ALIPMappingConstraint::ALIPMappingConstraint(
    const ConstrainedDynamicsInfo &dynamics) :
    NonlinearConstraint<double>(4, dynamics.nq() + dynamics.nv() + 4,
                                VectorXd::Zero(4), VectorXd::Zero(4)),
                                plant_(dynamics.get_plant()){
  context_ = plant_.CreateDefaultContext();
}

void ALIPMappingConstraint::EvaluateConstraint(
    const Eigen::Ref<const VectorXd>&x, VectorXd* y) const {

  const VectorXd qv = x.head(plant_.num_positions() + plant_.num_velocities());
  const VectorXd x_alip = x.tail<4>();

  plant_.SetPositionsAndVelocities(context_.get(), qv);

  const Vector3d com = plant_.CalcCenterOfMassPositionInWorld(*context_);
  const Vector3d foot = plant_.EvalBodyPoseInWorld(
      *context_, plant_.GetBodyByName(body_name_)) * point_;
  const Vector3d L =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, foot).rotational();

  *y = VectorXd::Zero(4);
  y->head<2>() = com.head<2>() - foot.head<2>() - x_alip.head<2>();
  y->tail<2>() = L.head<2>() - x_alip.tail<2>();
}

}