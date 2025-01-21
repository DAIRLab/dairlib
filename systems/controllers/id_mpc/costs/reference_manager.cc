#include "reference_manager.h"

namespace dairlib::systems::controllers::id_mpc {

template<typename T>
void ReferenceManager<T>::UpdateReference(
    const std::string& name,
    const drake::trajectories::Trajectory<double> &traj,
    const std::vector<double> &breaks) {
  // allow the full breaks vector to be passed in for convenience,
  // but only use the first N entries for input costs
  DRAKE_DEMAND(evaluators_.contains(name));
  DRAKE_DEMAND(evaluators_.at(name).size() <= breaks.size());

  const auto& evaluator_list = evaluators_.at(name);
  for (size_t i = 0; i < evaluator_list.size(); ++i) {
    evaluator_list[i]->UpdateReference(traj.value(breaks[i]));
  }
  if (terminal_evals_.contains(name)) {
    terminal_evals_.at(name)->UpdateReference(traj.value(breaks.back()));
  }
}

}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::controllers::id_mpc::ReferenceManager)