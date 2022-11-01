#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>

/*!
 * @brief Struct for a given mode as part of a mode sequence
 */
struct Mode{
  double start_phase;
  double end_phase;
  drake::VectorX<bool> contact_status; /// Vector describing which contacts are active
};

/*!
 * @brief Clss for defining a gait consisting of a vector of Modes, and a period for the whole gait.
 */
class Gait{
 public:
  Gait(const std::vector<Mode>& gait_pattern, double period = 0): gait_pattern_(gait_pattern),
  period_(period){};

/*!
 * @brief converts the gait into a trajectory from current time to end time
 * @param current_time
 * @param end_time
 * @return trajectory where value at time t converted to a bool is a vector of which feet are in stance
 */
  drake::trajectories::PiecewisePolynomial<double> ToTrajectory(double current_time, double end_time) const;

  /*!
   * @brief Checks to make sure the gait is valid, Asserts if not valid
   */
  void Is_Valid() const;

  void SetPeriod(double period){period_ = period;}

 private:
  /*!
   * @brief find the index for the current mode that the gait is in
   * @param time_now
   * @return
   */
  int CurrentMode(double time_now) const;

  double period_;
  std::vector<Mode> gait_pattern_; /// Vector of modes. Start phase of the first mode must be 0, end phase of the last mode
  /// must be 1, and no time gaps between start and end of sequential modes

};
