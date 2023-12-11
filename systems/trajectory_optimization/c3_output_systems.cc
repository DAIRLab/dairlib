#include "c3_output_systems.h"

#include "common/eigen_utils.h"
#include "solvers/c3_output.h"

#include "dairlib/lcmt_force.hpp"


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
      this->DeclareAbstractInputPort("J_lcs", drake::Value<MatrixXd>()).get_index();

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
    dairlib::lcmt_c3_forces* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
  const auto& J_c =
      this->EvalInputValue<MatrixXd>(context, lcs_contact_jacobian_port_);
  output_traj->num_forces = c3_solution->lambda_sol_.rows();
  output_traj->forces.resize(output_traj->num_forces);
  for (int i = 0; i < c3_solution->lambda_sol_.rows(); ++i){
    auto force = lcmt_force();
    force.contact_point[0] = 0;
    force.contact_point[1] = 0;
    force.contact_point[2] = 0;
    // 6, 7, 8 are the indices for the x,y,z components of the tray
    // TODO(yangwill): find a cleaner way to figure out the equivalent forces expressed in the world frame
    force.contact_force[0] = c3_solution->lambda_sol_(i, 0) * J_c->row(i)(6);
    force.contact_force[1] = c3_solution->lambda_sol_(i, 0) * J_c->row(i)(7);
    force.contact_force[2] = c3_solution->lambda_sol_(i, 0) * J_c->row(i)(8);
    output_traj->forces[i] = force;
  }
  output_traj->utime = context.get_time() * 1e6;
}


}  // namespace systems
}  // namespace dairlib