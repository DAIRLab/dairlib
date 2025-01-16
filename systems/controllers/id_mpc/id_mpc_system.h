#pragma once

#include "id_mpc.h"
#include "core/mpc_solution.h"

#include "drake/solvers/snopt_solver.h"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IDMPCSystem);

  IDMPCSystem(IDMPCParams params,
              std::unique_ptr<ConstrainedDynamicsInfo> dynamics);

 private:

  void SolveMPC(const drake::systems::Context<double>& context,
                MPCSolution* solution) const;

  // TODO (@Brian-Acosta) should this go straight to LCM?
  void CalcOutput(const drake::systems::Context<double>& context,
                  MPCSolution* solution) const;

  IDMPC mpc_problem_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_reference_;

  drake::systems::OutputPortIndex output_port_mpc_solution_;

  drake::systems::CacheIndex mpc_solution_cache_;
};

}