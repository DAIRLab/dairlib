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

// Adam's includes
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/vector_base.h>
#include <drake/math/rigid_transform.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/multibody/math/spatial_velocity.h>



using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {
namespace controllers {

class JIController : public LeafSystem<double> {
 public:
  JIController(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>& context,
      const Eigen::MatrixXd& K,
      const Eigen::MatrixXd& B);

  const drake::systems::InputPort<double>& get_input_port_config() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_input_port_output() const {
    return this->get_output_port(control_output_port_);
  }
  
  // void AddConstraint() const;
  // for PID
  void DoCalcTimeDerivatives(const Context<double>& context,
                             drake::systems::ContinuousState<double>* derivatives) const;
  
 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;

  int state_input_port_;
  int control_output_port_;
  const MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  const Eigen::MatrixXd K_;
  const Eigen::MatrixXd B_;

  // frame & point info
  const drake::multibody::BodyFrame<double>* EE_frame_;
  const drake::multibody::BodyFrame<double>* world_frame_;
  Eigen::Vector3d EE_offset_;

};

}  // namespace controller
}  // namespace systems
}  // namespace dairlib