#pragma once

#include <drake/common/yaml/yaml_io.h>

#include "common/file_utils.h"

#include "drake/common/yaml/yaml_read_archive.h"

enum ReducedModelTypes {
  kSupportDirections,
};

struct ReducedModelParams {
  ReducedModelTypes reduction_type;
  std::vector<Eigen::Vector3d> support_directions_vec;

  // Build as matrix type when serializing.
  Eigen::Matrix3Xd support_directions;

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, reduction_type);
    a->Visit(DRAKE_NVP(support_directions_vec));

    // Convert support_directions_vec to matrix.
    support_directions.resize(3, support_directions_vec.size());
    for (size_t i = 0; i < support_directions_vec.size(); i++) {
      support_directions.col(i) = support_directions_vec[i];
    }
  }
};
