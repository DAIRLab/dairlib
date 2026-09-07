#pragma once

// Configuration for the fast approximate jam label the sampling C3 controller
// computes per candidate sample.
//
// Every knob here is one field of systems::FastJammingLabelConfig
// (examples/sampling_c3/fast_jamming_label.h), which is where each one is
// documented -- what it trades, what it was measured to cost, and why it has
// the default it has.  Deliberately kept as a plain struct rather than
// embedding that config directly, so parameter_headers stays free of Drake
// plant code and every demo that loads controller params keeps linking what it
// links today.
//
// Presence of a risk_params_file in a demo's sampling_c3_controller_params.yaml
// is what turns the label on at all; a demo that omits the key computes no
// labels.

#include <string>
#include <vector>

#include "drake/common/yaml/yaml_read_archive.h"

struct SampleRiskParams {
  /// The SDFs of the objects the label's rollout simulates.  These are the
  /// SIMULATION's models (e.g. cone.sdf), not the controller's simplified LCS
  /// models (cone_controller.sdf) -- the label is an approximation of what the
  /// real sim would do, and the offline study that measured its agreement used
  /// the sim's geometry.  Exactly one object; the labeller requires a single
  /// free body.
  std::vector<std::string> object_models;

  /// FastJammingLabelConfig::sim_dt.
  double sim_dt;

  /// FastJammingLabelConfig::travel_threshold, in meters.
  double travel_threshold;

  /// FastJammingLabelConfig::settle_fraction.
  double settle_fraction;

  /// FastJammingLabelConfig::early_exit.
  bool early_exit;

  /// FastJammingLabelConfig::point_contact.
  bool point_contact;

  /// FastJammingLabelConfig::prescribed_ee.
  bool prescribed_ee;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(object_models));
    a->Visit(DRAKE_NVP(sim_dt));
    a->Visit(DRAKE_NVP(travel_threshold));
    a->Visit(DRAKE_NVP(settle_fraction));
    a->Visit(DRAKE_NVP(early_exit));
    a->Visit(DRAKE_NVP(point_contact));
    a->Visit(DRAKE_NVP(prescribed_ee));
  }
};
