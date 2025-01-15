#pragma once

#include "id_mpc.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IDMPCSystem);

  IDMPCSystem(IDMPCParams params,
              std::unique_ptr<ConstrainedDynamicsInfo> dynamics);

 private:
  IDMPC mpc_problem_;

  // TODO (Brian-Acosta) - Support more general reference
  //  trajectories, including task-space references
  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_reference_;

  drake::systems::OutputPortIndex output_port_mpc_debug_;
  drake::systems::OutputPortIndex output_port_mpc_solution_;
};

}