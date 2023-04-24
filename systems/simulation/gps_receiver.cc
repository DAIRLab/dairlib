#include "gps_receiver.h"
#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"

using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::VectorXd;


namespace dairlib::systems::simulation {


GpsReceiver::GpsReceiver(
    const MultibodyPlant<double>& plant, Context<double> *context,
    const Body<double> &body, const Vector3d &p_GB) :
    plant_(plant), context_(context), receiver_body_(body), p_GB_(p_GB) {
  DeclareVectorInputPort("x", plant_.num_positions() + plant.num_velocities());
  DeclareAbstractOutputPort("gps", &GpsReceiver::CalcReceiverPosition);
}

void GpsReceiver::CalcReceiverPosition(const Context<double>& context,
                                  lcmt_gps_signal* gps) const {
  const auto q = EvalVectorInput(context, 0)->get_value().head(plant_.num_positions());
  multibody::SetPositionsIfNew<double>(plant_, q, context_);
  auto X_WB = receiver_body_.EvalPoseInWorld(*context_);
  Vector3d pos_in_world = X_WB.translation() + X_WB.rotation().matrix() * p_GB_;

  for (int i = 0; i < 3; i++) {
    gps->receiver_pos_in_world[i] = pos_in_world(i);
    gps->receiver_pos_in_parent_body[i] = p_GB_(i);
  }
  gps->parent_body_name = receiver_body_.name();
  gps->mtime = static_cast<long>(1000 * context.get_time());
  gps->cov = 1e-3;
}

}