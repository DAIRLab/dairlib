#pragma once

#include <memory>

#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

#define DRIVE_FILTER_NB 9
#define JOINT_FILTER_NB 4
#define JOINT_FILTER_NA 3

static int drive_filter_b[DRIVE_FILTER_NB] = {2727, 534, -2658, -795, 72,
                                              110,  19,  -6,    -3};

static double joint_filter_b[JOINT_FILTER_NB] = {12.348, 12.348, -12.348,
                                                 -12.348};

static double joint_filter_a[JOINT_FILTER_NA] = {1.0, -1.7658, 0.79045};

/// this class is copied from drake/systems/primitives/Passthrough
/// with the modification that it adds random noise to the values it is
/// passing through
class Encoder final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Encoder)

  /// Constructs a pass through system (`y = u.segment(start,length)`).
  /// @param num_positions the number of generalized positions
  /// @param num_velocities the number of generalized velocities
  /// @param num_inputs the number of inputs
  /// @param pos_variance covariance matrix for the generalized position
  /// @param vel_variance covariance matrix for the generalized velocities
  explicit Encoder(const drake::multibody::MultibodyPlant<double>& plant,
                   std::vector<int>& joint_indices,
                   std::vector<int>& ticks_per_revolution);

  ~Encoder() override {}

  /// Returns the sole input port.
  const drake::systems::InputPort<double>& get_input_port() const {
    return drake::systems::LeafSystem<double>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const drake::systems::OutputPort<double>& get_output_port() const {
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 protected:
  void UpdateFilter(const drake::systems::Context<double>& context,
                    systems::BasicVector<double>* output) const;

 private:
  struct DriveFilter {
    int x[DRIVE_FILTER_NB];
  };
  struct JointFilter {
    double x[JOINT_FILTER_NB];
    double y[JOINT_FILTER_NA];
  };

  bool is_abstract() const { return false; }

  int num_positions_;
  int num_velocities_;
  std::vector<int> joint_indices_;
  std::vector<int> ticks_per_revolution_;
  std::vector<JointFilter> joint_filters_;
};

}  // namespace systems
}  // namespace dairlib
