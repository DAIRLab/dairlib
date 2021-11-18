#pragma once
#include "systems/controllers/osc/options_tracking_data.h"

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"

namespace dairlib {
namespace systems {
namespace controllers {

using goldilocks_models::ReducedOrderModel;

class OptimalRomTrackingData final : public OptionsTrackingData {
 public:
  OptimalRomTrackingData(
      const std::string& name, int n_y, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

  void AddRom(const goldilocks_models::ReducedOrderModel& rom);
  void AddStateAndRom(int state,
                      const goldilocks_models::ReducedOrderModel& rom);

 private:
  void UpdateY(const Eigen::VectorXd& x_wo_spr,
               const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_wo_spr,
                  const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               const drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   const drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // TODO: not sure why compile error when I used ReducedOrderModel reference
  //  instead of pointer
  std::unordered_map<int, const goldilocks_models::ReducedOrderModel*>
      rom_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib