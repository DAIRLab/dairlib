#pragma once

#include <drake/common/yaml/yaml_io.h>

#include "common/file_utils.h"

#include "drake/common/yaml/yaml_read_archive.h"

/* Reduction type options:
    0. kSupportDirections:  grabs support points along specified directions. */
enum ReducedModelTypes {
  kSupportDirections,
};

/* Methods for spring constant computation from tetrahedral mesh:
    0. kVanGelder:  "Approximate simulation of elastic membranes by triangulated
       spring meshes" 1998
    1. kLloydEtAl:  "Identification of spring parameters for deformable object
       simulation" 2007 */
enum SpringConstantMethods {
  kVanGelder,
  kLloydEtAl,
};

struct ReducedModelParams {
  ReducedModelTypes reduction_type;
  std::vector<Eigen::Vector3d> support_directions_vec;
  std::vector<Eigen::Vector4i> tetrahedra_vec;
  SpringConstantMethods spring_constant_method;
  double youngs_modulus;
  double yield_stress;
  Eigen::Vector3d object_half_widths;

  // Build as matrix type when serializing.
  Eigen::Matrix3Xd support_directions;
  Eigen::Matrix4Xi tetrahedra;

  template <typename Archive>
  void Serialize(Archive* a) {
    ENUM_DESERIALIZE(a, reduction_type);
    ENUM_DESERIALIZE(a, spring_constant_method);
    a->Visit(DRAKE_NVP(support_directions_vec));
    a->Visit(DRAKE_NVP(tetrahedra_vec));
    a->Visit(DRAKE_NVP(youngs_modulus));
    a->Visit(DRAKE_NVP(yield_stress));
    a->Visit(DRAKE_NVP(object_half_widths));

    // Convert vectors to matrices.
    support_directions.resize(3, support_directions_vec.size());
    for (size_t i = 0; i < support_directions_vec.size(); i++) {
      support_directions.col(i) = support_directions_vec[i];
    }
    tetrahedra.resize(4, tetrahedra_vec.size());
    for (size_t i = 0; i < tetrahedra_vec.size(); i++) {
      tetrahedra.col(i) = tetrahedra_vec[i];
    }

    // Perform checking on the compatibility of the support directions and
    // tetrahedra.
    for (int i = 0; i < tetrahedra.cols(); i++) {
      for (int j = 0; j < 4; j++) {
        int idx = tetrahedra(j, i);
        if (idx < 0 || idx >= support_directions.cols()) {
          throw std::runtime_error(
              "Tetrahedra indices are out of bounds of support directions.");
        }
      }
    }
  }
};
