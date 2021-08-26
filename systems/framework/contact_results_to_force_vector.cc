#include "systems/framework/contact_results_to_force_vector.h"

#include <drake/common/eigen_types.h>

namespace dairlib {
namespace systems {

using drake::multibody::ContactResults;
using drake::systems::BasicVector;
using drake::systems::kUseDefaultName;
using Eigen::MatrixXd;
using Eigen::VectorXd;

ContactResultsToForceVector::ContactResultsToForceVector(
    const drake::multibody::MultibodyPlant<double>& plant)
    : drake::systems::LeafSystem<double>() {
  DRAKE_DEMAND(plant.is_finalized());
  const int body_count = plant.num_bodies();

  body_names_.reserve(body_count);
  using std::to_string;
  for (drake::multibody::BodyIndex i{0}; i < body_count; ++i) {
    const drake::multibody::Body<double>& body = plant.get_body(i);
    body_names_.push_back(body.name() + "(" + to_string(body.model_instance()) +
                          ")");
    for (auto geometry_id : plant.GetCollisionGeometriesForBody(body))
      geometry_id_to_body_name_map_[geometry_id] = body.name();
  }

  contact_result_input_port_index_ =
      this->DeclareAbstractInputPort(systems::kUseDefaultName,
                                     drake::Value<ContactResults<double>>())
          .get_index();
  contact_forces_output_port_index_ =
      this->DeclareVectorOutputPort(
              kUseDefaultName, BasicVector<double>(12),
              &ContactResultsToForceVector::ConvertContactResultsToVectorOutput)
          .get_index();
}

void ContactResultsToForceVector::ConvertContactResultsToVectorOutput(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  const auto& contact_results =
      get_contact_result_input_port().template Eval<ContactResults<double>>(
          context);
  VectorXd output_vector = VectorXd::Zero(12);
  MatrixXd contact_forces = MatrixXd::Zero(4, 3);
  MatrixXd contact_info_locs = MatrixXd::Zero(4, 3);
  int num_left_contacts = 0;
  int num_right_contacts = 0;
  for (int i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    const drake::multibody::PointPairContactInfo<double>& contact_info =
        contact_results.point_pair_contact_info(i);

    //    info_msg.body1_name = body_names_.at(contact_info.bodyA_index());
    //    info_msg.body2_name = body_names_.at(contact_info.bodyB_index());
    if (body_names_.at(contact_info.bodyB_index()) == "toe_left(2)") {
      if (num_left_contacts >= 2) continue;
      contact_info_locs.row(num_left_contacts) = contact_info.contact_point();
      contact_forces.row(num_left_contacts) = contact_info.contact_force();
      num_left_contacts += 1;
    } else if (body_names_.at(contact_info.bodyB_index()) == "toe_right(2)") {
      if (num_right_contacts >= 2) continue;
      contact_info_locs.row(num_right_contacts) = contact_info.contact_point();
      contact_forces.row(num_right_contacts) = contact_info.contact_force();
      num_right_contacts += 1;
    }
  }
  while (num_left_contacts != 2) {
    contact_forces.row(num_left_contacts) = VectorXd::Zero(3);
    contact_info_locs.row(num_left_contacts) = VectorXd::Zero(3);
    num_left_contacts += 1;
  }
  while (num_right_contacts != 2) {
    contact_forces.row(2 + num_right_contacts) = VectorXd::Zero(3);
    contact_info_locs.row(2 + num_right_contacts) = VectorXd::Zero(3);
    num_right_contacts += 1;
  }

  for (int i = 0; i < 4; ++i) {
    if (contact_info_locs(0, 0) > contact_info_locs(1, 0)) {
      output_vector.segment(0, 3) = contact_forces.row(1);
      output_vector.segment(3, 3) = contact_forces.row(0);
    } else {
      output_vector.segment(0, 3) = contact_forces.row(0);
      output_vector.segment(3, 3) = contact_forces.row(1);
    }
    if (contact_info_locs(0, 0) > contact_info_locs(1, 0)) {
      output_vector.segment(6, 3) = contact_forces.row(3);
      output_vector.segment(9, 3) = contact_forces.row(2);
    } else {
      output_vector.segment(6, 3) = contact_forces.row(2);
      output_vector.segment(9, 3) = contact_forces.row(3);
    }
  }
//  std::cout << output_vector << std::endl;
  output->SetFromVector(output_vector);
}

}  // namespace systems
}  // namespace dairlib
