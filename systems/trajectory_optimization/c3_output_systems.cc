#include "c3_output_systems.h"

#include "common/eigen_utils.h"
#include "dairlib/lcmt_force.hpp"
#include "solvers/c3_output.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::VectorXd;

C3OutputSender::C3OutputSender() {
  c3_solution_port_ =
      this->DeclareAbstractInputPort("c3_solution",
                                     drake::Value<C3Output::C3Solution>{})
          .get_index();
  c3_intermediates_port_ =
      this->DeclareAbstractInputPort("c3_intermediates",
                                     drake::Value<C3Output::C3Intermediates>{})
          .get_index();
  lcs_contact_jacobian_port_ =
      this->DeclareAbstractInputPort("J_lcs", drake::Value<MatrixXd>())
          .get_index();
  lcs_contact_points_port_ =
      this->DeclareAbstractInputPort("p_lcs",
                                     drake::Value<std::vector<VectorXd>>())
          .get_index();

  this->set_name("c3_output_sender");
  lcm_c3_output_port_ = this->DeclareAbstractOutputPort(
                                "lcmt_c3_output", dairlib::lcmt_c3_output(),
                                &C3OutputSender::OutputC3Lcm)
                            .get_index();
  lcs_forces_output_port_ = this->DeclareAbstractOutputPort(
                                    "lcmt_c3_force", dairlib::lcmt_c3_forces(),
                                    &C3OutputSender::OutputC3Forces)
                                .get_index();
//  lcs_inputs_output_port_ =
//      this->DeclareVectorOutputPort("u_lcs",
//                                    drake::systems::BasicVector<double>(3),
//                                    &C3OutputSender::OutputNextC3Input)
//          .get_index();
}

void C3OutputSender::OutputC3Lcm(const drake::systems::Context<double>& context,
                                 dairlib::lcmt_c3_output* output) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
  const auto& c3_intermediates =
      this->EvalInputValue<C3Output::C3Intermediates>(context,
                                                      c3_intermediates_port_);

  C3Output c3_output = C3Output(*c3_solution, *c3_intermediates);
  *output = c3_output.GenerateLcmObject(context.get_time());
}

void C3OutputSender::OutputC3Forces(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_c3_forces* c3_forces_output) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
  const auto& J_c =
      this->EvalInputValue<MatrixXd>(context, lcs_contact_jacobian_port_);
  const auto& contact_points = this->EvalInputValue<std::vector<VectorXd>>(
      context, lcs_contact_points_port_);
  c3_forces_output->num_forces = c3_solution->lambda_sol_.rows();
  c3_forces_output->forces.resize(c3_forces_output->num_forces);
  int forces_per_contact = J_c->rows() / contact_points->size();
  int contact_var_start;
  for (int contact_index = 0; contact_index < contact_points->size();
       ++contact_index) {
    contact_var_start = forces_per_contact * contact_index;
    for (int i = 0; i < forces_per_contact; ++i) {
      auto force = lcmt_force();
      force.contact_point[0] = contact_points->at(contact_index)[0];
      force.contact_point[1] = contact_points->at(contact_index)[1];
      force.contact_point[2] = contact_points->at(contact_index)[2];
      // 6, 7, 8 are the indices for the x,y,z components of the tray
      // TODO(yangwill): find a cleaner way to figure out the equivalent forces
      // expressed in the world frame
      force.contact_force[0] =
          c3_solution->lambda_sol_(contact_var_start + i, 0) *
          J_c->row(contact_var_start + i)(6);
      force.contact_force[1] =
          c3_solution->lambda_sol_(contact_var_start + i, 0) *
          J_c->row(contact_var_start + i)(7);
      force.contact_force[2] =
          c3_solution->lambda_sol_(contact_var_start + i, 0) *
          J_c->row(contact_var_start + i)(8);
      c3_forces_output->forces[contact_var_start + i] = force;
    }
  }
  c3_forces_output->utime = context.get_time() * 1e6;
}

// void C3OutputSender::OutputNextC3Input(const drake::systems::Context<double>&
// context,
//                  drake::systems::BasicVector<double>* u_next) const {
//  const auto& c3_solution =
//      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
//  u_next->SetFromVector(c3_solution->u_sol_.col(0).cast<double>());
//}

}  // namespace systems
}  // namespace dairlib