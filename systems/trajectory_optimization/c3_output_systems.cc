#include "c3_output_systems.h"


#include "common/eigen_utils.h"
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

  this->set_name("c3_output_sender");
  lcm_c3_output_port_ = this->DeclareAbstractOutputPort(
                                "lcmt_c3_output", dairlib::lcmt_c3_output(),
                                &C3OutputSender::OutputC3Lcm)
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

}  // namespace systems
}  // namespace dairlib