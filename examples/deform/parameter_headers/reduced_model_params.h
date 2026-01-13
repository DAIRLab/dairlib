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
  std::vector<Eigen::Vector2i> connections_vec;

  // Build as matrix type when serializing.
  Eigen::Matrix3Xd support_directions;
  Eigen::Matrix2Xi connections;

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, reduction_type);
    a->Visit(DRAKE_NVP(support_directions_vec));
    a->Visit(DRAKE_NVP(connections_vec));

    // Convert vectors to matrices.
    support_directions.resize(3, support_directions_vec.size());
    for (size_t i = 0; i < support_directions_vec.size(); i++) {
      support_directions.col(i) = support_directions_vec[i];
    }
    connections.resize(2, connections_vec.size());
    for (size_t i = 0; i < connections_vec.size(); i++) {
      connections.col(i) = connections_vec[i];
    }

    // Perform checking on the compatibility of the support directions and
    // connections.
    for (int i = 0; i < connections.cols(); i++) {
      int idx1 = connections(0, i);
      int idx2 = connections(1, i);
      if (idx1 < 0 || idx1 >= support_directions.cols() || idx2 < 0 ||
          idx2 >= support_directions.cols()) {
        throw std::runtime_error(
            "Connection indices are out of bounds of support directions.");
      }
    }
  }
};
