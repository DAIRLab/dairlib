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
  lcs_contact_info_port_ =
      this->DeclareAbstractInputPort("J_lcs, p_lcs", drake::Value<std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>>())
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
  const auto& contact_info =
      this->EvalInputValue<std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>>>(context, lcs_contact_info_port_);
  MatrixXd J_c = contact_info->first;
  int contact_force_start = c3_solution->lambda_sol_.rows() - J_c.rows();
  auto contact_points = contact_info->second;
  int forces_per_contact = contact_info->first.rows() / contact_points.size();
  c3_forces_output->num_forces = forces_per_contact * contact_points.size();
  c3_forces_output->forces.resize(c3_forces_output->num_forces);

  int contact_var_start;
  int contact_jacobian_row_start;
  for (int contact_index = 0; contact_index < contact_points.size();
       ++contact_index) {
    contact_var_start = contact_force_start + forces_per_contact * contact_index;
    contact_jacobian_row_start = forces_per_contact * contact_index;
    for (int i = 0; i < forces_per_contact; ++i) {
      int force_row = contact_jacobian_row_start + i;
      if (contact_force_start > 0){
        if (i == 0){
          force_row = contact_index;
        }else{
          force_row = contact_points.size() + (forces_per_contact - 1) * contact_index + i;
        }
      }
      auto force = lcmt_force();
      force.contact_point[0] = contact_points.at(contact_index)[0];
      force.contact_point[1] = contact_points.at(contact_index)[1];
      force.contact_point[2] = contact_points.at(contact_index)[2];
      // 6, 7, 8 are the indices for the x,y,z components of the tray
      // TODO(yangwill): find a cleaner way to figure out the equivalent forces
      // expressed in the world frame
      force.contact_force[0] =
          c3_solution->lambda_sol_(contact_var_start + i, 0) *
          J_c.row(force_row)(6);
      force.contact_force[1] =
          c3_solution->lambda_sol_(contact_var_start + i, 0) *
          J_c.row(force_row)(7);
      force.contact_force[2] =
          c3_solution->lambda_sol_(contact_var_start + i, 0) *
          J_c.row(force_row)(8);
      c3_forces_output->forces[contact_jacobian_row_start + i] = force;
    }
  }
  c3_forces_output->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib