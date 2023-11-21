#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class PlateBalancingTargetGenerator
    : public drake::systems::LeafSystem<double> {
 public:
  PlateBalancingTargetGenerator();

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_end_effector_target() const {
    return this->get_output_port(end_effector_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_tray_target()
      const {
    return this->get_output_port(tray_target_port_);
  }

  void SetRemoteControlParameters(const Eigen::Vector3d& neutral_pose,
                                  double x_scale, double y_scale,
                                  double z_scale);

 private:
  void CalcEndEffectorTarget(const drake::systems::Context<double>& context,
                             drake::systems::BasicVector<double>* target) const;
  void CalcTrayTarget(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* target) const;

  drake::systems::InputPortIndex radio_port_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  drake::systems::OutputPortIndex tray_target_port_;

  Eigen::Vector3d neutral_pose_;
  double x_scale_;
  double y_scale_;
  double z_scale_;
  //  std::vector<Eigen::Vector4d> half_plane_bounds_;
};

}  // namespace systems
}  // namespace dairlib