#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/controllers/id_mpc/references/mpc_reference.h"


namespace dairlib::systems::controllers::id_mpc {

/*!
 * Class to hold onto an MPC reference with constant trajectories,
 * and output that reference with the time shifted so that the MPC knots
 * start at the current time
 */
class ConstantReferenceSystem : public drake::systems::LeafSystem<double> {
 public:
  ConstantReferenceSystem(const MPCReference& reference);

 private:
  void CalcOutput(const drake::systems::Context<double>& context,
                  MPCReference* out) const;

  const MPCReference reference_;

};

}