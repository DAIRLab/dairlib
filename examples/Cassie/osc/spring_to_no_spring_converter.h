#pragma once

#include <map>
#include <string>
#include <vector>

#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {

class SpringToNoSpringConverter : public drake::systems::LeafSystem<double> {
 public:
  explicit SpringToNoSpringConverter(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

 private:
  void CopyOutput(const drake::systems::Context<double>& context,
                  systems::OutputVector<double>* output) const;

  int nq_wo_spr_;
  int nv_wo_spr_;
  int nu_wo_spr_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;
};

}  // namespace cassie
}  // namespace dairlib
