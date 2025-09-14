#pragma once

#include <vector>

#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "common/update_context.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_controller_params.h"
#include "examples/sampling_c3/sampling_c3_utils.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_options.h"
#include "solvers/c3_plus.h"
#include "solvers/c3_qp.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"
#include "systems/controllers/sampling_based_c3_controller.h"

namespace dairlib {

class PushAnythingSolverBenchmarker {
 public:
  explicit PushAnythingSolverBenchmarker(bool verbose = false);

  void Solve(const Eigen::VectorXd& x_lcs_curr,
             const Eigen::VectorXd& x_lcs_des);
  std::vector<double> GetQPSolveTimes() { return qp_solve_times_; }
  std::vector<double> GetProjectionSolveTimes() {
    return projection_solve_times_;
  }

 private:
  SamplingC3ControllerParams controller_params_;
  SamplingC3Options sampling_c3_options_;
  C3Options c3_options_;

  drake::multibody::MultibodyPlant<double>* plant_lcs_;
  std::unique_ptr<drake::systems::Diagram<double>> plant_lcs_diagram_;
  std::unique_ptr<drake::systems::Context<double>> diagram_context_;
  drake::systems::Context<double>* plant_lcs_context_;
  std::unique_ptr<drake::multibody::MultibodyPlant<drake::AutoDiffXd>>
      plant_lcs_ad_;
  std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>> context_ad_;

  std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>
      contact_pairs_;
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms_;

  int n_q_{0};
  int n_v_{0};
  int n_u_{0};
  int n_x_{0};
  double dt_{0.0};
  int N_{0};
  bool verbose_{false};

  solvers::ContactModel contact_model_ = solvers::ContactModel::kStewartAndTrinkle;


  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> G_;
  std::vector<Eigen::MatrixXd> U_;

  drake::solvers::SolverOptions solver_options_;

  std::vector<double> qp_solve_times_;
  std::vector<double> projection_solve_times_;
};

}  // namespace dairlib
