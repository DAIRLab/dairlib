#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>
#include "systems/framework/timestamped_vector.h"

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "examples/iC3/iC3_options.h"
#include "drake/systems/framework/leaf_system.h"
#include "multibody/multibody_utils.h"
#include "c3/core/c3_options.h"
#include "c3/core/lcs.h"
#include "c3/multibody/lcs_factory.h"
#include "c3/multibody/multibody_utils.h"
#include "c3/systems/c3_controller_options.h"
#include "systems/framework/timestamped_vector.h"
#include "examples/iC3/iC3_options.h"

using dairlib::LcmTrajectory;
using dairlib::lcmt_timestamped_saved_traj;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::vector;
using drake::systems::Context;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::AutoDiffXd;
using drake::systems::BasicVector;
using dairlib::systems::TimestampedVector;

namespace dairlib {

class C3GoalGenerator : public drake::systems::LeafSystem<double> {
  public:

    // example idx 0 = plate
    // example idx 1 = trifinger
    explicit C3GoalGenerator(
        MultibodyPlant<double>& plant,
        c3::multibody::LCSFactory lcs_factory,
        iC3Options ic3_options,
        c3::systems::C3ControllerOptions c3_controller_options, 
        VectorXd x_des, int example_idx);

    const drake::systems::InputPort<double>& get_input_port_state() const {
      return this->get_input_port(state_port_);
    }
    const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
      return this->get_input_port(nominal_position_port_);
    }
    const drake::systems::InputPort<double>& get_input_port_ic3_x() const {
      return this->get_input_port(ic3_x_port_);
    }
    const drake::systems::InputPort<double>& get_input_port_timestep() const {
      return this->get_input_port(timestep_port_);
    }
    const drake::systems::OutputPort<double>& get_output_port_target() const {
      return this->get_output_port(target_port_);
    }
    const drake::systems::OutputPort<double>& get_output_port_lcs() const {
      return this->get_output_port(lcs_port_);
    }
    const drake::systems::OutputPort<double>& get_output_port_x_curr() const {
      return this->get_output_port(x_lcs_port_);
    }


  private:
    void OutputTarget(const Context<double>& context,
                      BasicVector<double>* target) const;
    void OutputState(const Context<double>& context,
                      TimestampedVector<double>* state_output) const;
    void OutputLCS(const Context<double>& context,
                  c3::LCS* lcs) const;
    c3::LCS MakeTimeVaryingLCS(MatrixXd x_hat, MatrixXd u_hat) const;

    drake::systems::InputPortIndex state_port_;
    drake::systems::InputPortIndex nominal_position_port_;
    drake::systems::InputPortIndex ic3_x_port_;
    drake::systems::InputPortIndex timestep_port_;

    drake::systems::OutputPortIndex target_port_;
    drake::systems::OutputPortIndex lcs_port_;
    drake::systems::OutputPortIndex x_lcs_port_;

    const drake::multibody::MultibodyPlant<double>& plant_;

    mutable c3::multibody::LCSFactory lcs_factory_;
    iC3Options ic3_options_;

    c3::C3Options c3_options_;
    c3::systems::C3ControllerOptions c3_controller_options_;
    VectorXd x_des_;

    std::vector<int> quaternion_indices_;

    int N_;

    int n_q_;
    int n_v_;
    int n_x_;
    int n_u_;

    int example_idx_;
    
    int n_lambda_;

};

}  // namespace dairlib
