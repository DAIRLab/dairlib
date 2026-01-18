#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "examples/cube_flip/parameter_headers/iC3_options.h"
#include "drake/systems/framework/leaf_system.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_options.h"
#include "solvers/lcs.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/framework/timestamped_vector.h"

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
using dairlib::solvers::LCS;

namespace dairlib {

class C3GoalGenerator : public drake::systems::LeafSystem<double> {
  public:
    explicit C3GoalGenerator(
        MultibodyPlant<double>& plant,
        Context<double>* context,
        MultibodyPlant<AutoDiffXd>& plant_ad,
        Context<AutoDiffXd>* context_ad,
        vector<SortedPair<GeometryId>>& contact_geoms,
        C3Options c3_options, VectorXd x_des);

    const drake::systems::InputPort<double>& get_input_port_state() const {
      return this->get_input_port(state_port_);
    }
    const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
      return this->get_input_port(nominal_position_port_);
    }
    const drake::systems::OutputPort<double>& get_output_port_target() const {
      return this->get_output_port(target_port_);
    }
    const drake::systems::OutputPort<double>& get_output_port_lcs() const {
      return this->get_output_port(lcs_port_);
    }


  private:
    void OutputTarget(const Context<double>& context,
                      BasicVector<double>* target) const;

    void OutputLCS(const Context<double>& context,
                  LCS* lcs) const;

    drake::systems::InputPortIndex state_port_;
    drake::systems::InputPortIndex nominal_position_port_;

    drake::systems::OutputPortIndex target_port_;
    drake::systems::OutputPortIndex lcs_port_;

    drake::multibody::MultibodyPlant<double>& plant_;
    Context<double>* context_;
    MultibodyPlant<AutoDiffXd>& plant_ad_;
    Context<AutoDiffXd>* context_ad_;
    vector<SortedPair<GeometryId>>& contact_geoms_;
    C3Options c3_options_;
    VectorXd x_des_;

    int N_;

    int n_q_;
    int n_v_;
    int n_x_;
    int n_u_;

    int n_lambda_;
    solvers::ContactModel contact_model_;

};

}  // namespace dairlib
