#pragma once

#include <memory>

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

#define CASSIE_ENC_RES_LOW 8192
#define CASSIE_ENC_RES_HIGH 262144

#define CASSIE_JOINT_FILTER_NB 4
#define CASSIE_JOINT_FILTER_NA 3

static double joint_filter_b[CASSIE_JOINT_FILTER_NB] = {12.348, 12.348, -12.348,
                                                 -12.348};

static double joint_filter_a[CASSIE_JOINT_FILTER_NA] = {1.0, -1.7658, 0.79045};

/// Class to capture the quantization effects of Cassie's encoders
/// The resolution and velocity filter values are taken from the supplied MuJoCo
/// simulator from Agility Robotics
class CassieEncoder final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieEncoder)

  explicit CassieEncoder(const drake::multibody::MultibodyPlant<double>& plant,
                         std::vector<int>& joint_pos_indices,
                         std::vector<int>& joint_vel_indices,
                         std::vector<int>& ticks_per_revolution);

  ~CassieEncoder() override = default;

 protected:
  void UpdateFilter(const drake::systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

 private:
  /// Unused: this filter is for the measured motor torques which are not being
  /// used
  struct JointFilter {
    double x[CASSIE_JOINT_FILTER_NB];
    double y[CASSIE_JOINT_FILTER_NA];
  };

  bool is_abstract() const { return false; }

  int num_positions_;
  int num_velocities_;
  std::vector<int> joint_pos_indices_;
  std::vector<int> joint_vel_indices_;
  std::vector<int> ticks_per_revolution_;
  std::vector<std::unique_ptr<JointFilter>> joint_filters_;
};

}  // namespace dairlib
