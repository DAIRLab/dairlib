#pragma once

#include <vector>
#include <utility>
#include <chrono>
#include <assert.h>


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

// Adam's includes
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/math/rigid_transform.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/multibody/math/spatial_velocity.h>

#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"



using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RotationMatrix;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;

namespace dairlib {
namespace systems {
namespace controllers {

class ImpedanceController : public LeafSystem<double> {
 public:
  ImpedanceController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::MultibodyPlant<double>& plant_contact,
      drake::systems::Context<double>& context,
      drake::systems::Context<double>& context_contact,
      const Eigen::MatrixXd& K,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& K_null,
      const Eigen::MatrixXd& B_null,
      const Eigen::VectorXd& qd,
      const std::vector<drake::geometry::GeometryId>& contact_geoms,
      int num_friction_directions);

  const drake::systems::InputPort<double>& get_input_port_config() const {
    return this->get_input_port(franka_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3() const {
    return this->get_input_port(c3_state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_input_port_output() const {
    return this->get_output_port(control_output_port_);
  }
  
 private:
  // computes the control input
  void CalcControl(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const;
  // computes the rotational error of the rotational matrix R w.r.t orientation_d_
  Vector3d CalcRotationalError(const RotationMatrix<double>& R) const;
  // computes the contact jacobians in J_n and J_t
  void CalcContactJacobians(const std::vector<SortedPair<GeometryId>>& contact_pairs,
                    VectorXd& phi, MatrixXd& J_n, MatrixXd& J_t) const;

  // ports
  int franka_state_input_port_;
  int c3_state_input_port_;
  int control_output_port_;
  
  // constructor variables
  const MultibodyPlant<double>& plant_;
  const MultibodyPlant<double>& plant_f_;
  drake::systems::Context<double>& context_;
  drake::systems::Context<double>& context_f_;
  const Eigen::MatrixXd K_;
  const Eigen::MatrixXd B_;
  const MatrixXd K_null_;
  const MatrixXd B_null_;
  const VectorXd qd_;
  std::vector<drake::geometry::GeometryId> contact_geoms_;
  const int num_friction_directions_;

  // frame, EE, and contact info
  const drake::multibody::BodyFrame<double>* EE_frame_;
  const drake::multibody::BodyFrame<double>* world_frame_;
  Eigen::Vector3d EE_offset_;
  std::vector<SortedPair<GeometryId>> contact_pairs_;
  int n_; // franka DoF = 7

  // control related variables
  Quaterniond orientation_d_;

};

}  // namespace controller
}  // namespace systems
}  // namespace dairlib