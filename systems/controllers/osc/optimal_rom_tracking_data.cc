#include "optimal_rom_tracking_data.h"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers {

/**** OptimalRomTrackingData ****/
OptimalRomTrackingData::OptimalRomTrackingData(
    const string& name, int n_y, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OptionsTrackingData(name, n_y, n_y, K_p, K_d, W, plant_w_spr,
                          plant_wo_spr) {
  this->use_only_plant_wo_spr_in_evaluation_ = true;
}

void OptimalRomTrackingData::AddRom(
    const goldilocks_models::ReducedOrderModel& rom) {
  DRAKE_DEMAND(GetYDim() == rom.n_y());
  AddStateAndRom(-1, rom);
}

void OptimalRomTrackingData::AddStateAndRom(
    int state, const goldilocks_models::ReducedOrderModel& rom) {
  AddFiniteStateToTrack(state);
  AddRom(rom);
}

void OptimalRomTrackingData::UpdateY(const VectorXd& x_wo_spr,
                                     const Context<double>& context_wo_spr) {
  y_ = rom_.at(fsm_state_)
           ->EvalMappingFunc(x_wo_spr.head(plant_wo_spr_.num_positions()),
                             context_wo_spr);
}
void OptimalRomTrackingData::UpdateYdot(const VectorXd& x_wo_spr,
                                        const Context<double>& context_wo_spr) {
  ydot_ = rom_.at(fsm_state_)
              ->EvalMappingFuncJV(x_wo_spr.head(plant_wo_spr_.num_positions()),
                                  x_wo_spr.tail(plant_wo_spr_.num_velocities()),
                                  context_wo_spr);
}
void OptimalRomTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                     const Context<double>& context_wo_spr) {
  J_ = rom_.at(fsm_state_)
           ->EvalMappingFuncJ(x_wo_spr.head(plant_wo_spr_.num_positions()),
                              context_wo_spr);
}
void OptimalRomTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ =
      rom_.at(fsm_state_)
          ->EvalMappingFuncJdotV(x_wo_spr.head(plant_wo_spr_.num_positions()),
                                 x_wo_spr.tail(plant_wo_spr_.num_velocities()),
                                 context_wo_spr);
}
void OptimalRomTrackingData::CheckDerivedOscTrackingData() {}

}  // namespace dairlib::systems::controllers