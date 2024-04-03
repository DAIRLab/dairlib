#include "external_force_generator.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::EventStatus;

namespace dairlib {

ExternalForceGenerator::ExternalForceGenerator(
    drake::multibody::BodyIndex body_index)
    : body_index_(body_index) {
  // Input/Output Setup
  radio_port_ =
      this->DeclareVectorInputPort("lcmt_radio_out", BasicVector<double>(18))
          .get_index();

  spatial_force_port_ =
      this->DeclareAbstractOutputPort(
              "object_spatial_force",
              std::vector<
                  drake::multibody::ExternallyAppliedSpatialForce<double>>(),
              &ExternalForceGenerator::CalcSpatialForce)
          .get_index();
}

void ExternalForceGenerator::SetRemoteControlParameters(double x_scale,
                                                        double y_scale,
                                                        double z_scale) {
  x_scale_ = x_scale;
  y_scale_ = y_scale;
  z_scale_ = z_scale;
}

void ExternalForceGenerator::CalcSpatialForce(
    const drake::systems::Context<double>& context,
    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>*
        spatial_forces) const {
  const auto& radio_out = this->EvalVectorInput(context, radio_port_);
  Vector3d force = VectorXd::Zero(3);
  if (radio_out->value()[12]) {
    force(0) = radio_out->value()[0] * x_scale_;
    force(1) = radio_out->value()[1] * y_scale_;
    force(2) = radio_out->value()[2] * z_scale_;
  }
  if (spatial_forces->empty()) {
    auto spatial_force =
        drake::multibody::ExternallyAppliedSpatialForce<double>();
    spatial_force.body_index = body_index_;
    spatial_force.F_Bq_W =
        drake::multibody::SpatialForce<double>(Vector3d::Zero(), force);
    spatial_forces->push_back(spatial_force);
  } else {
    spatial_forces->at(0).F_Bq_W =
        drake::multibody::SpatialForce<double>(Vector3d::Zero(), force);
    spatial_forces->at(0).body_index = body_index_;
  }
}

}  // namespace dairlib
