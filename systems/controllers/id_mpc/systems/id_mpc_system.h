#pragma once

#include "mpc_solution.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/id_mpc/id_mpc.h"
#include "systems/controllers/id_mpc/sqp/sqp_solver.h"

#include "drake/solvers/snopt_solver.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IDMPCSystem);

  IDMPCSystem(IDMPCParams params,
              std::unique_ptr<ConstrainedDynamicsInfo> dynamics);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }

  const drake::systems::InputPort<double>& get_input_port_reference() const {
    return get_input_port(input_port_reference_);
  }

  const ConstrainedDynamicsInfo& dynamics() const {
    return trajopt_.dynamics();
  }

  IDMPC* get_mutable_trajopt_ptr() {
    return &trajopt_;
  }

 private:

  void SetInitialSolverStateToCurrent(
      const OutputVector<double>& x_u_t,
      const std::vector<std::vector<std::string>>& contacts,
      SQPIterate& solver_state) const;

  void SetInitialSolverStateToReference(
      const MPCReference& reference, SQPIterate& solver_state) const;

  void SetInitialSolverStateBlended(
      const OutputVector<double>& x_u_t,
      const MPCReference& reference, SQPIterate& solver_state) const;

  drake::systems::EventStatus
  SolveMPC(const drake::systems::Context<double>&context,
                drake::systems::State<double>* state) const;

  // TODO (@Brian-Acosta) should this go straight to LCM?
  void CalcOutput(const drake::systems::Context<double>& context,
                  lcmt_timestamped_saved_traj* solution) const;

  mutable IDMPC trajopt_;
  mutable SQPSolver solver_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_reference_;

  drake::systems::OutputPortIndex output_port_mpc_solution_;
  drake::systems::AbstractStateIndex mpc_solution_state_;
};

}