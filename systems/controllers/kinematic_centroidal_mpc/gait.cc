#include "gait.h"

int Gait::CurrentMode(double time_now) const{
  double phase_now = fmod(time_now/period_, 1);
  for(int i = 0; i < gait_pattern_.size(); i++){
    const auto& mode = gait_pattern_[i];
    if(mode.start_phase <= phase_now && mode.end_phase > phase_now){
      return i;
    }
  }
  DRAKE_ASSERT(false);
  return 0;
}

drake::trajectories::PiecewisePolynomial<double> Gait::ToTrajectory(double current_time, double end_time) const {
  std::vector<double> break_points;
  std::vector<drake::MatrixX<double>> samples;

  // Calculate initial mode index, and phase
  int current_mode = CurrentMode( current_time);
  double current_phase = fmod(current_time/period_, 1);

  // Loop until time is greater than end time
  while(current_time < end_time){
    // Add the break point for the current time
    break_points.push_back(current_time);
    samples.emplace_back(gait_pattern_[current_mode].contact_status.cast<double>());

    // Update time based on how much longer the current mode will last
    current_time += (gait_pattern_[current_mode].end_phase - current_phase) * period_;
    // Update the mode and mod if necessary
    current_mode = (current_mode + 1) % gait_pattern_.size();
    // The new phase is the start phase of the updated mode
    current_phase = gait_pattern_[current_mode].start_phase;
  }
  break_points.push_back(end_time);
  samples.push_back(samples[samples.size()-1]);
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(break_points, samples);
}

void Gait::Is_Valid() const {
  DRAKE_ASSERT(period_ > 0);
  DRAKE_ASSERT(gait_pattern_[0].start_phase == 0);
  DRAKE_ASSERT(gait_pattern_[gait_pattern_.size()-1].end_phase == 1);
  for(int i = 0; i < gait_pattern_.size() - 1; i++){
    DRAKE_ASSERT(gait_pattern_[i].end_phase == gait_pattern_[i+1].start_phase);
  }
}
