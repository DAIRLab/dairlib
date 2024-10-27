#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"


namespace dairlib::systems {

class NotchFilter : public drake::systems::LeafSystem<double> {

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NotchFilter);
  NotchFilter(size_t vector_size, double stop_freq_low, double stop_freq_high,
              size_t num_taps);

 private:

  void CalcOutput(const drake::systems::Context<double>& context,
                  dairlib::systems::TimestampedVector<double>* out) const;

  void SetDefaultState(const drake::systems::Context<double>& context,
                       drake::systems::State<double>* state) const final;

  drake::systems::EventStatus Update(const drake::systems::Context<double>& context,
                                     drake::systems::State<double>* state) const;


  drake::systems::AbstractStateIndex buffer_idx_;
  drake::systems::AbstractStateIndex coefficients_idx_;
  drake::systems::AbstractStateIndex prev_timestamp_idx_;
  drake::systems::AbstractStateIndex sampling_rate_idx_;

  const size_t num_taps_;
  const double f0_;
  const double f1_;
};


}

