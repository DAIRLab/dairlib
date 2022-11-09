#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"

/*!
 * @brief Struct for a given mode as part of a mode sequence
 */
struct Mode{
  double start_phase;
  double end_phase;
  drake::VectorX<bool> contact_status; /// Vector describing which contacts are active

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(start_phase));
    a->Visit(DRAKE_NVP(end_phase));
    a->Visit(DRAKE_NVP(contact_status));
  }
};

/*!
 * @brief Struct for defining a gait consisting of a vector of Modes, and a period for the whole gait.
 */
struct Gait{
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
  void check_valid() const;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(gait_pattern));
    a->Visit(DRAKE_NVP(period));
  }
  /*!
   * @brief find the index for the current mode that the gait is in
   * @param time_now
   * @return
   */
  int GetCurrentMode(double time_now) const;

  double period;
  std::vector<Mode> gait_pattern; /// Vector of modes. Start phase of the first mode must be 0, end phase of the last mode
  /// must be 1, and no time gaps between start and end of sequential modes

};
