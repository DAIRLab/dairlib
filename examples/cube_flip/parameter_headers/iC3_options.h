#pragma once
#include "drake/common/yaml/yaml_read_archive.h"

struct iC3Options {

  int num_iters;
  bool add_position_constraints;
  int num_segments;

  bool print_costs;

  int iter_to_use;

  int N;
  double dt; // REMOVE after importing all options from c3 repo

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_iters));
    a->Visit(DRAKE_NVP(add_position_constraints));
    a->Visit(DRAKE_NVP(num_segments));
    a->Visit(DRAKE_NVP(print_costs));
    a->Visit(DRAKE_NVP(iter_to_use));
    a->Visit(DRAKE_NVP(N)); // REMOVE after importing all options from c3 repo
    a->Visit(DRAKE_NVP(dt)); // REMOVE after importing all options from c3 repo

  }
};
