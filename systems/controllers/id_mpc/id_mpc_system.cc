#include "id_mpc_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers::id_mpc {

IDMPCSystem::IDMPCSystem(IDMPCParams params,
                         std::unique_ptr<ConstrainedDynamicsInfo> dynamics) :
                         mpc_problem_(params, std::move(dynamics)) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(mpc_problem_.dynamics().get_plant())
  ).get_index();

  input_port_reference_ = DeclareVectorInputPort(
      "xd", mpc_problem_.dynamics().nx()
  ).get_index();
}



}