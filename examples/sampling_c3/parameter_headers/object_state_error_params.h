#pragma once

#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <optional>

#include "drake/common/yaml/yaml_read_archive.h"

/// Parameters describing the object state estimation errors injected into
/// simulation so that the simulated object pose estimate resembles what the
/// hardware pose estimator produces.
///
/// The defaults are derived from the 2026-09-17 hardware logs
/// (~/3d_printer/logs/2026/09_17_26/00000{0,1}).  Those show the error is
/// two-scale:  a sub-millimeter white noise floor plus rare, sustained,
/// multi-second excursions an order of magnitude larger.  The three terms below
/// (bias, white noise, drift) can each be enabled independently so a term can
/// be switched off without deleting its tuned values.
struct ObjectStateErrorParams {
  /// Seed for the error random number generator.  If unset, the generator is
  /// seeded from std::random_device and runs are not reproducible.
  std::optional<int> seed;

  /// Constant offset, drawn once at construction from these per-axis standard
  /// deviations and held for the whole run.  Models a miscalibrated object
  /// frame.
  bool enable_bias;
  Eigen::VectorXd bias_position_std;         // m, size 3.
  Eigen::VectorXd bias_orientation_std_deg;  // deg, size 3.

  /// Independent Gaussian redrawn at every object state publish.  This is the
  /// measured hardware noise floor.
  bool enable_white_noise;
  Eigen::VectorXd white_noise_position_std;         // m, size 3.
  Eigen::VectorXd white_noise_orientation_std_deg;  // deg, size 3.

  /// Ornstein-Uhlenbeck process reproducing the sustained excursions.  The
  /// standard deviations are the *stationary* standard deviations, i.e. the
  /// long-run 1-sigma of the drift, not a per-step increment.
  bool enable_drift;
  Eigen::VectorXd drift_position_std;         // m, size 3.
  Eigen::VectorXd drift_orientation_std_deg;  // deg, size 3.
  double drift_time_constant;                 // s.

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(seed));

    a->Visit(DRAKE_NVP(enable_bias));
    a->Visit(DRAKE_NVP(bias_position_std));
    a->Visit(DRAKE_NVP(bias_orientation_std_deg));

    a->Visit(DRAKE_NVP(enable_white_noise));
    a->Visit(DRAKE_NVP(white_noise_position_std));
    a->Visit(DRAKE_NVP(white_noise_orientation_std_deg));

    a->Visit(DRAKE_NVP(enable_drift));
    a->Visit(DRAKE_NVP(drift_position_std));
    a->Visit(DRAKE_NVP(drift_orientation_std_deg));
    a->Visit(DRAKE_NVP(drift_time_constant));

    auto check = [](const char* field_name, const Eigen::VectorXd& std_devs) {
      if (std_devs.size() != 3) {
        throw std::runtime_error(std::string(field_name) + " has " +
                                 std::to_string(std_devs.size()) +
                                 " entries but must have 3 (one per axis).");
      }
      if (std_devs.minCoeff() < 0.0) {
        throw std::runtime_error(std::string(field_name) +
                                 " has a negative standard deviation.");
      }
    };
    if (enable_bias) {
      check("bias_position_std", bias_position_std);
      check("bias_orientation_std_deg", bias_orientation_std_deg);
    }
    if (enable_white_noise) {
      check("white_noise_position_std", white_noise_position_std);
      check("white_noise_orientation_std_deg", white_noise_orientation_std_deg);
    }
    if (enable_drift) {
      check("drift_position_std", drift_position_std);
      check("drift_orientation_std_deg", drift_orientation_std_deg);
      if (drift_time_constant <= 0.0) {
        throw std::runtime_error(
            "drift_time_constant must be positive when enable_drift is true.");
      }
    }
  }
};
