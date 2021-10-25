#include <drake/systems/primitives/linear_system.h>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems {

/// FiniteHorizonLqrSwingFootTrajGenerator generates a trajectory for the
/// swing foot to determine the suitable swing foot acceleration in the xy
/// directions. A trajectory is then calculated such that y_error = 0,
/// ydot_error = 0, and y_ddot = -K(t)(y - yd) where yd is the xy coordinates
/// of the next stance foot location. A cubic spline is used for the swing foot
/// z coordinate

typedef struct SwingFootTajGenOptions {
  std::string floating_base_body_name = "";
  double double_support_duration;
  double mid_foot_height;
  double desired_final_foot_height ;
  double desired_final_vertical_foot_velocity;
  double max_com_to_x_footstep_dist;
  double footstep_offset;
  double center_line_offset;
  bool wrt_com_in_local_frame = false;
} SwingFootTajGenOptions;

class FiniteHorizonLqrSwingFootTrajGenerator :
     public drake::systems::LeafSystem<double> {
 public:
  FiniteHorizonLqrSwingFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::vector<int> left_right_support_fsm_states,
      const std::vector<double> left_right_support_durations,
      const std::vector<std::pair<const Eigen::Vector3d,
                            const drake::multibody::Frame<double>&>> pts,
      const SwingFootTajGenOptions opts);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const {
    return this->get_input_port(liftoff_time_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_com() const {
    return this->get_input_port(com_port_);
  }

 private:

  // MBP parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  drake::systems::LinearSystem<double> double_integrator_;
  mutable std::unique_ptr<drake::systems::Context<double>> double_integrator_context_;

  // Footstep parameters
  const SwingFootTajGenOptions opts_;
  const std::vector<int> left_right_support_fsm_states_;
  const std::vector<double> left_right_support_durations_;
  const std::vector<std::pair<const Eigen::Vector3d,
                              const drake::multibody::Frame<double>&>> pts_;

  // ports
  int state_port_;
  int fsm_port_;
  int liftoff_time_port_;
  int com_port_;

  // LQR parameters
  Eigen::Matrix<double, 4, 4> Q_;
  Eigen::Matrix<double, 2, 2> R_;
};
}