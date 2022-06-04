#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"

#include "drake/systems/framework/leaf_system.h"
//#include
//"external/drake/common/_virtual_includes/autodiff/drake/common/eigen_autodiff_types.h"
//#include
//"external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/systems/framework/context.h"
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/lcs_factory.h"

#include "drake/common/autodiff.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/optimization/manipulator_equation_constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {
namespace controllers {

class C3Controller_franka : public LeafSystem<double> {
 public:
  C3Controller_franka(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::multibody::MultibodyPlant<double>& plant_f,
      const drake::multibody::MultibodyPlant<double>& plant_franka,
      drake::systems::Context<double>& context,
      drake::systems::Context<double>& context_f,
      drake::systems::Context<double>& context_franka,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_f,
      drake::systems::Context<drake::AutoDiffXd>& context_ad,
      drake::systems::Context<drake::AutoDiffXd>& context_ad_f,
      const drake::geometry::SceneGraph<double>& scene_graph,
      const drake::systems::Diagram<double>& diagram,
      std::vector<drake::geometry::GeometryId> contact_geoms,
      int num_friction_directions, double mu,
      const std::vector<Eigen::MatrixXd>& Q,
      const std::vector<Eigen::MatrixXd>& R,
      const std::vector<Eigen::MatrixXd>& G,
      const std::vector<Eigen::MatrixXd>& U,
      const std::vector<Eigen::VectorXd>& xdesired,
      const drake::trajectories::PiecewisePolynomial<double>& pp);

  const drake::systems::InputPort<double>& get_input_port_config() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_input_port_output() const {
    return this->get_output_port(state_output_port_);

  }


  // void AddConstraint() const;

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;

  int state_input_port_;
  int state_output_port_;
  const MultibodyPlant<double>& plant_;
  MultibodyPlant<double>& plant_f_;
  const MultibodyPlant<double>& plant_franka_;
  drake::systems::Context<double>& context_;
  drake::systems::Context<double>& context_f_;
  drake::systems::Context<double>& context_franka_;
  const MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  MultibodyPlant<drake::AutoDiffXd>& plant_ad_f_;
  drake::systems::Context<drake::AutoDiffXd>& context_ad_;
  drake::systems::Context<drake::AutoDiffXd>& context_ad_f_;
  const drake::geometry::SceneGraph<double>& scene_graph_;
  const drake::systems::Diagram<double>& diagram_;
  std::vector<drake::geometry::GeometryId> contact_geoms_;
  int num_friction_directions_;
  double mu_;
  const std::vector<Eigen::MatrixXd> Q_;
  const std::vector<Eigen::MatrixXd> R_;
  const std::vector<Eigen::MatrixXd> G_;
  const std::vector<Eigen::MatrixXd> U_;
  const std::vector<Eigen::VectorXd> xdesired_;
  const drake::trajectories::PiecewisePolynomial<double> pp_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib