#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_lqr_output.hpp"
#include "lcm/lcm_trajectory.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream> 

using drake::multibody::MultibodyPlant;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

namespace dairlib {

// Outputs a manually generated set of inputs
class LQRInput : public drake::systems::LeafSystem<double> {
 public:
  explicit LQRInput(const MultibodyPlant<double>& plant, int N, double dt, int iter_to_use, double time_to_wait);

  const drake::systems::InputPort<double>& get_input_port_ic3_x() const {
    return this->get_input_port(input_port_ic3_x_);
  }

  const drake::systems::InputPort<double>& get_input_port_x_curr() const {
    return this->get_input_port(input_port_x_curr_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_u() const {
    return this->get_input_port(input_port_u_);
  }

  const drake::systems::InputPort<double>& get_input_port_lqr() const {
    return this->get_input_port(input_port_lqr_);
  }

  const drake::systems::OutputPort<double>& get_output_port_u() const {
    return this->get_output_port(output_port_index_);
  }


 private:
  drake::systems::InputPortIndex input_port_x_curr_;
  drake::systems::InputPortIndex input_port_ic3_x_;
  drake::systems::InputPortIndex input_port_u_;
  drake::systems::InputPortIndex input_port_lqr_;

  drake::systems::OutputPortIndex output_port_index_;

  const MultibodyPlant<double>& plant_;
  double dt_; 
  int N_;
  int iter_to_use_;
  double time_to_wait_;

  void ComputeLQRInput(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const;


};

} // namespace dairlib