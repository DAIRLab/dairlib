#pragma once

#include "mpc_solution.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/id_mpc/walking/id_mpc_walking.h"
#include "systems/controllers/id_mpc/sqp/nc_sqp_solver.h"

#include "drake/solvers/snopt_solver.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_id_mpc_walking_debug.hpp"

namespace dairlib::systems::controllers::id_mpc {

class IDMPCWalkingSystem : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IDMPCWalkingSystem);

  IDMPCWalkingSystem(
      IDMPCParams params, std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
      GaitParams gait_params);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return get_input_port(input_port_state_);
  }

  const drake::systems::InputPort<double>& get_input_port_reference() const {
    return get_input_port(input_port_reference_);
  }

  const drake::systems::OutputPort<double>& get_output_port_mpc_solution()
  const {
    return get_output_port(output_port_mpc_solution_);
  }

  const drake::systems::OutputPort<double>& get_output_port_mpc_debug() const {
    return get_output_port(output_port_mpc_debug_);
  }

  const ConstrainedDynamicsInfo& dynamics() const {
    return trajopt_.mpc().dynamics();
  }

  IDMPC* get_mutable_trajopt_ptr() {
    IDMPC& mpc = trajopt_.mutable_mpc();
    return &mpc;
  }

 private:

  std::vector<Eigen::Vector3d> CalcInitialFootsteps(
      const Eigen::VectorXd& q, const MPCReference& ref) const;

  std::vector<Eigen::Vector3d> CalcFootstepLocations(
      const MPCReference& reference, const Eigen::VectorXd& z) const;

  void SetInitialSolverStateToCurrent(
      const OutputVector<double>& x_u_t,
      const std::vector<std::vector<std::string>>& contacts,
      SQPIterate& solver_state) const;

  void ShiftSolution(const std::vector<double>& knots,
                     MPCSolution* prev_sol) const;

  drake::systems::EventStatus
  SolveMPC(const drake::systems::Context<double>&context,
           drake::systems::State<double>* state) const;

  void CalcOutput(const drake::systems::Context<double>& context,
                  lcmt_timestamped_saved_traj* solution) const;

  void CalcDebug(const drake::systems::Context<double>& context,
                 lcmt_id_mpc_walking_debug* debug) const;

  mutable IDMPCWalking trajopt_;
  mutable NCSQPSolver solver_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_reference_;

  drake::systems::OutputPortIndex output_port_mpc_solution_;
  drake::systems::OutputPortIndex output_port_mpc_debug_;
  drake::systems::AbstractStateIndex mpc_solution_state_;
};

}