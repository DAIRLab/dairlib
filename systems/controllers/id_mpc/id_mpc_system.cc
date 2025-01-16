#include "id_mpc_system.h"
#include "systems/framework/output_vector.h"
#include "costs/mpc_reference.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Context;

IDMPCSystem::IDMPCSystem(IDMPCParams params,
                         std::unique_ptr<ConstrainedDynamicsInfo> dynamics) :
                         mpc_problem_(params, std::move(dynamics)) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(mpc_problem_.dynamics().get_plant())
  ).get_index();

  input_port_reference_ = DeclareAbstractInputPort(
      "mpc_reference",
      drake::Value<MPCReference>()).get_index();

  mpc_solution_cache_ = DeclareCacheEntry(
      "mpc_solution", MPCSolution(),
      &IDMPCSystem::SolveMPC).cache_index();

  output_port_mpc_solution_ = DeclareAbstractOutputPort(
      "mpc_solution", MPCSolution(),
      &IDMPCSystem::CalcOutput).get_index();
}

void IDMPCSystem::SolveMPC(
    const Context<double> &context, MPCSolution *solution) const {

}

void IDMPCSystem::CalcOutput(const Context<double>& context,
                             MPCSolution *solution) const {

}

}