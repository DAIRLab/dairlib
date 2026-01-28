#include "mpm_model_reducer.h"

#include <algorithm>
#include <iostream>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_material_points.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace systems {

using drake::systems::Context;
using Eigen::Matrix2Xi;
using Eigen::Matrix3Xd;
using Eigen::Matrix4d;
using Eigen::Matrix4Xi;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Converts MPM points to LCM type lcmt_tetrahedra.
MpmPointsToTetrahedra::MpmPointsToTetrahedra(
    ReducedModelParams reduction_params)
    : reduction_type_(reduction_params.reduction_type),
      n_support_directions_(reduction_params.support_directions.cols()),
      support_directions_(reduction_params.support_directions),
      n_fixed_tetrahedra_(reduction_params.tetrahedra.cols()),
      fixed_tetrahedra_(reduction_params.tetrahedra) {
  if (reduction_type_ != ReducedModelTypes::kSupportDirections) {
    throw std::runtime_error(
        "Only support direction reduction is currently implemented.");
  }

  this->set_name("MpmPointsToTetrahedra");

  lcmt_material_points_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_material_points",
              drake::Value<dairlib::lcmt_material_points>{})
          .get_index();

  lcmt_tetrahedra_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_tetrahedra", dairlib::lcmt_tetrahedra(),
              &MpmPointsToTetrahedra::OutputTetrahedraLcm)
          .get_index();
}

void MpmPointsToTetrahedra::OutputTetrahedraLcm(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_tetrahedra* output) const {
  // Evaluate input port to get the MPM contents.
  const auto& material_points_lcmt =
      this->EvalInputValue<dairlib::lcmt_material_points>(
          context, lcmt_material_points_input_port_);

  // Check if it's too early to output or if there are no points.
  Matrix3Xd reduced_points = Matrix3Xd::Zero(3, 0);
  Matrix4Xi tetrahedra = Matrix4Xi::Zero(4, 0);
  if ((material_points_lcmt->utime > 1e-3) &&
      (material_points_lcmt->num_points > 1)) {
    // Extract the MPM points.
    int n_points = material_points_lcmt->num_points;
    Matrix3Xd points = Matrix3Xd::Zero(3, n_points);
    for (int point_i = 0; point_i < n_points; point_i++) {
      std::vector<float> point = material_points_lcmt->points[point_i];
      for (int dim_i = 0; dim_i < 3; dim_i++) {
        points(dim_i, point_i) = point[dim_i];
      }
    }

    // From the MPM points and the reduction method, compute the reduced points.
    if (reduction_type_ == ReducedModelTypes::kSupportDirections) {
      std::pair<Matrix3Xd, Matrix4Xi> reduced_points_and_tetrahedra =
          ComputeTetrahedraFromSupportFunction(points);
      reduced_points = reduced_points_and_tetrahedra.first;
      tetrahedra = reduced_points_and_tetrahedra.second;
    }
  }

  // Prepare data for LCM publishing.
  int n_reduced_points = reduced_points.cols();
  int n_tetrahedra = tetrahedra.cols();
  std::vector<std::vector<float>> points_data(n_reduced_points,
                                              std::vector<float>(3, 0));
  for (int i = 0; i < n_reduced_points; i++) {
    for (int j = 0; j < 3; j++) {
      points_data[i][j] = reduced_points.col(i)(j);
    }
  }
  std::vector<std::vector<int>> tetrahedra_data(n_tetrahedra,
                                                std::vector<int>(4, 0));
  for (int i = 0; i < n_tetrahedra; i++) {
    for (int j = 0; j < 4; j++) {
      tetrahedra_data[i][j] = tetrahedra.col(i)(j);
    }
  }

  // Set the fields of the LCM message.
  output->utime = context.get_time() * 1e6;
  output->num_points = n_reduced_points;
  output->num_tetrahedra = n_tetrahedra;
  output->points = points_data;
  output->tetrahedra = tetrahedra_data;
}

std::pair<Matrix3Xd, Matrix4Xi>
MpmPointsToTetrahedra::ComputeTetrahedraFromSupportFunction(
    const Matrix3Xd& mpm_points) const {
  // Compute the support point along each support direction.
  Matrix3Xd reduced_points = Matrix3Xd::Zero(3, n_support_directions_);
  Matrix4Xi tetrahedra = fixed_tetrahedra_;
  for (int dir_i = 0; dir_i < n_support_directions_; dir_i++) {
    VectorXd projections =
        support_directions_.col(dir_i).transpose() * mpm_points;
    Eigen::Index max_index;
    projections.maxCoeff(&max_index);
    reduced_points.col(dir_i) = mpm_points.col(max_index);
  }
  return std::make_pair(reduced_points, tetrahedra);
}

/// Converts tetrahedra representing an approximately volumetrically homogeneous
/// solid to LCM type lcmt_elastoplastic_network.
TetrahedraToElastoPlasticNetwork::TetrahedraToElastoPlasticNetwork(
    double youngs_modulus, double yield_stress,
    SpringConstantMethods spring_constant_method)
    : youngs_modulus_(youngs_modulus),
      yield_stress_(yield_stress),
      spring_constant_method_(spring_constant_method) {
  this->set_name("TetrahedraToElastoPlasticNetwork");

  lcmt_tetrahedra_input_port_ =
      this->DeclareAbstractInputPort("lcmt_tetrahedra",
                                     drake::Value<dairlib::lcmt_tetrahedra>{})
          .get_index();

  lcmt_elastoplastic_network_output_port_ =
      this->DeclareAbstractOutputPort(
              "lcmt_elastoplastic_network",
              dairlib::lcmt_elastoplastic_network(),
              &TetrahedraToElastoPlasticNetwork::OutputElastoPlasticNetworkLcm)
          .get_index();
}

void TetrahedraToElastoPlasticNetwork::OutputElastoPlasticNetworkLcm(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_elastoplastic_network* output) const {
  // Evaluate input port to get the tetrahedra.
  const auto& tetrahedra_lcmt = this->EvalInputValue<dairlib::lcmt_tetrahedra>(
      context, lcmt_tetrahedra_input_port_);

  // Check if it's too early to output or if there are no points.
  int n_points = 0;
  int n_connections = 0;
  Matrix3Xd points = Matrix3Xd::Zero(3, n_points);
  std::map<std::pair<int, int>, std::pair<double, double>> ks_and_ls_map;
  if ((tetrahedra_lcmt->utime > 1e-3) && (tetrahedra_lcmt->num_points > 1)) {
    // Extract the vertices.
    n_points = tetrahedra_lcmt->num_points;
    points = Matrix3Xd::Zero(3, n_points);
    for (int point_i = 0; point_i < n_points; point_i++) {
      std::vector<float> point = tetrahedra_lcmt->points[point_i];
      for (int dim_i = 0; dim_i < 3; dim_i++) {
        points(dim_i, point_i) = point[dim_i];
      }
    }

    // Compute volumes, edge lengths, and spring constant contributions for each
    // tetrahedron.
    int n_tetrahedra = tetrahedra_lcmt->num_tetrahedra;
    for (int tet_i = 0; tet_i < n_tetrahedra; tet_i++) {
      std::vector<int> tet = tetrahedra_lcmt->tetrahedra[tet_i];

      // Compute the volume of the tetrahedron.
      Matrix4d vertices_ones = Matrix4d::Ones();
      for (int i = 0; i < 4; i++) {
        vertices_ones.block<3, 1>(0, i) = points.col(tet[i]);
      }
      double volume = std::abs(vertices_ones.determinant()) / 6.0;

      // For each edge in the tetrahedron, add a contribution to each
      // connection's spring constant.
      // A tetrahedron's contribution to an edge's spring constant is:
      // k_spring += (E * V) / L_spring^2            <-- van Gelder's approach
      // k_spring += 2*sqrt(2)/25 * E * l_equiv      <-- Lloyd et al.'s approach
      // ...where l_equiv = (12/sqrt(2) * V)^(1/3)
      for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
          double length = (points.col(tet[i]) - points.col(tet[j])).norm();
          double spring_coeff = 0;
          if (spring_constant_method_ == SpringConstantMethods::kVanGelder) {
            spring_coeff = youngs_modulus_ * volume / std::pow(length, 2);
          } else if (spring_constant_method_ ==
                     SpringConstantMethods::kLloydEtAl) {
            double l_equiv =
                std::pow(12.0 / std::sqrt(2.0) * volume, 1.0 / 3.0);
            spring_coeff =
                (2.0 * std::sqrt(2.0) / 25.0) * youngs_modulus_ * l_equiv;
          } else {
            throw std::runtime_error(
                "Unknown spring constant computation method.");
          }
          auto key = std::make_pair(std::min(tet[i], tet[j]),
                                    std::max(tet[i], tet[j]));
          if (ks_and_ls_map.find(key) == ks_and_ls_map.end()) {
            ks_and_ls_map[key] = std::make_pair(spring_coeff, length);
          } else {
            ks_and_ls_map[key].first += spring_coeff;
          }
        }
      }
    }
  }

  // Convert the connection graph to a list of connections, spring constants,
  // and yield forces.
  n_connections = ks_and_ls_map.size();
  std::vector<std::vector<int>> connections_data(n_connections,
                                                 std::vector<int>(2, 0));
  std::vector<double> spring_constants =
      std::vector<double>(n_connections, 0.0);
  std::vector<double> yield_forces = std::vector<double>(n_connections, 0.0);
  int conn_i = 0;
  for (const auto& connections_ks_ls : ks_and_ls_map) {
    connections_data[conn_i][0] = connections_ks_ls.first.first;
    connections_data[conn_i][1] = connections_ks_ls.first.second;
    spring_constants[conn_i] = connections_ks_ls.second.first;
    // Yield stress and Young's modulus are related by:
    // yield_stress = youngs_modulus * yield_strain
    // ... where yield_strain is given by:
    // yield_strain = dl_yield / l
    // ... and in the spring network, we have:
    // f_yield = k_spring * dl_yield
    // Therefore:
    // f_yield = k_spring * l * yield_stress / youngs_modulus
    yield_forces[conn_i] = connections_ks_ls.second.first *
                           connections_ks_ls.second.second * yield_stress_ /
                           youngs_modulus_;
    conn_i++;
  }

  // Convert the Eigen matrices to std::vectors.
  std::vector<std::vector<float>> points_data(n_points,
                                              std::vector<float>(3, 0));
  for (int i = 0; i < n_points; i++) {
    for (int j = 0; j < 3; j++) {
      points_data[i][j] = points.col(i)(j);
    }
  }

  // Set the fields of the LCM message.
  output->utime = context.get_time() * 1e6;
  output->num_points = n_points;
  output->num_connections = n_connections;
  output->points = points_data;
  output->connections = connections_data;
  output->spring_constants = spring_constants;
  output->yield_forces = yield_forces;
}

}  // namespace systems
}  // namespace dairlib
