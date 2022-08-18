#pragma once

#include <vector>
#include <map>
#include <string>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"

#include "drake/systems/framework/leaf_system.h"
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "common/find_resource.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/lcs_factory_franka.h"

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

#include "examples/franka_trajectory_following/c3_parameters.h"
#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_io.h"
#include <random>


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

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;
  void StateEstimation(Eigen::VectorXd& q_plant, Eigen::VectorXd& v_plant,
                       const Eigen::Vector3d end_effector, double timestamp) const;
  Eigen::Vector3d ProjectStateEstimate(
      const Eigen::Vector3d& endeffector, const Eigen::Vector3d& estimate) const;

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
  C3Parameters param_;
  std::map<string, int> q_map_franka_;
  std::map<string, int> v_map_franka_;
  std::map<string, int> q_map_;
  std::map<string, int> v_map_;
  const double PI{3.14159265359};
  const double max_desired_velocity_{0.5}; //0.3

  // dt filter
  // TODO: make all the mutable variables drake states
  mutable std::deque<double> moving_average_;
  mutable double prev_timestamp_;
  uint32_t dt_filter_length_;

  // velocity
  mutable Eigen::Vector3d prev_position_;
  mutable Eigen::Vector3d prev_velocity_;
  mutable std::deque<Eigen::Vector3d> past_velocities_;

  mutable bool received_first_message_{false};
  mutable double first_message_time_{-1.0};

  // warm starting
  mutable std::vector<VectorXd> warm_start_x_;
  mutable std::vector<VectorXd> warm_start_lambda_;
  mutable std::vector<VectorXd> warm_start_u_;
  mutable std::vector<VectorXd> warm_start_delta_;
  mutable std::vector<VectorXd> warm_start_binary_;

  // kalman filter
  // mutable VectorXd xhat_prev;
  // mutable MatrixXd P_prev;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib