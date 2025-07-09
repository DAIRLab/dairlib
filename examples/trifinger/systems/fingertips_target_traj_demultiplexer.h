#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {
class FingertipsTargetTrajDemultiplexer
    : public drake::systems::LeafSystem<double> {
 public:
  FingertipsTargetTrajDemultiplexer(drake::systems::Context<double>* context);

  const drake::systems::InputPort<double>& get_input_port_traj() const {
    return this->get_input_port(input_traj_port_);
  }
  const drake::systems::OutputPort<double>&
  get_output_port_fingertip_0_target_traj() const {
    return this->get_output_port(fingertip_0_target_traj_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_fingertip_120_target_traj() const {
    return this->get_output_port(fingertip_120_target_traj_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_fingertip_240_target_traj() const {
    return this->get_output_port(fingertip_240_target_traj_port_);
  }

 private:
  void CalcFingertip0TargetTraj(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* target_traj) const {
    this->CopyToOutput(context, 0, 3, target_traj);
  }
  void CalcFingertip120TargetTraj(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* target_traj) const {
    this->CopyToOutput(context, 3, 3, target_traj);
  }
  void CalcFingertip240TargetTraj(
      const drake::systems::Context<double>& context,
      drake::trajectories::Trajectory<double>* target_traj) const {
    this->CopyToOutput(context, 6, 3, target_traj);
  }
  void CopyToOutput(const drake::systems::Context<double>& context,
                    const int start_index, const int size,
                    drake::trajectories::Trajectory<double>* target_traj) const;

  drake::systems::Context<double>* context_;

  drake::systems::InputPortIndex input_traj_port_;
  drake::systems::OutputPortIndex fingertip_0_target_traj_port_;
  drake::systems::OutputPortIndex fingertip_120_target_traj_port_;
  drake::systems::OutputPortIndex fingertip_240_target_traj_port_;
};
}  // namespace dairlib::systems