#pragma once

#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_lqr_output.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include "lcm/lcm_trajectory.h"
#include <c3/core/lcs.h>
#include <c3/systems/framework/c3_output.h>
#include <c3/systems/c3_controller_options.h>

#include "examples/iC3/iC3_options.h"

#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using dairlib::lcmt_radio_out;
using drake::SortedPair;
using drake::geometry::GeometryId;
using std::vector;

namespace dairlib {
namespace systems {

class iC3LqrTrackingController : public drake::systems::LeafSystem<double> {
 public:
  explicit iC3LqrTrackingController(const drake::multibody::MultibodyPlant<double>& plant,  
            iC3Options ic3_options, MatrixXd R, int N, int example_idx); 

  const drake::systems::InputPort<double>& get_input_port_target() const {
    return this->get_input_port(target_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs_state() const {
    return this->get_input_port(lcs_state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lqr() const {
    return this->get_input_port(lqr_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_x() const {
    return this->get_input_port(ic3_x_port_);
  }
  
  const drake::systems::InputPort<double>& get_input_port_ic3_u() const {
    return this->get_input_port(ic3_u_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_lambda() const {
    return this->get_input_port(ic3_lambda_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs() const {
    return this->get_input_port(lcs_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_timestep() const {
    return this->get_input_port(timestep_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_actor_input() const {
    return this->get_output_port(actor_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_tracking_target()
      const {
    return this->get_output_port(tracking_target_port_);
  }

 private:
  c3::LCS CreatePlaceholderLCS() const;

  drake::systems::EventStatus ComputePlan(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void OutputActorInput(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* actor_input) const;

  void OutputTrackingTarget(const drake::systems::Context<double>& context,
                        drake::systems::BasicVector<double>* tracking_target) const;

  int GetNearestXForValueFunction(
      Eigen::VectorXd x_curr, Eigen::MatrixXd x_hat_slice) const;

  drake::systems::InputPortIndex target_input_port_;
  drake::systems::InputPortIndex lcs_state_input_port_;
  drake::systems::InputPortIndex lcs_input_port_;
  drake::systems::InputPortIndex lqr_input_port_;
  drake::systems::InputPortIndex ic3_x_port_;
  drake::systems::InputPortIndex ic3_u_port_;
  drake::systems::InputPortIndex ic3_lambda_port_;
  drake::systems::InputPortIndex timestep_port_;
  
  drake::systems::OutputPortIndex actor_port_;
  drake::systems::OutputPortIndex tracking_target_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;

  iC3Options ic3_options_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  MatrixXd R_;
  int example_idx_;

  mutable VectorXd tracking_target_;
  mutable VectorXd u_out_;

  int N_; // N for c3 (NOT the same as iC3 horizon)

};

}  // namespace systems
}  // namespace dairlib
