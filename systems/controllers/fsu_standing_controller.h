#pragma once

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers{

class StandingPelvisPD: public drake::systems::LeafSystem<double> {
 public:
  StandingPelvisPD(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context);

  const drake::systems::InputPort<double>& get_input_port_state_desired_() const {
    return this->get_input_port(state_port_desired_);
  }

  const drake::systems::InputPort<double>& get_input_port_state_actual_() const{
      return this->get_input_port(state_port_robot_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_force_output() const{
      return this->get_output_port(output_forces);
  }

 private:
  Eigen::VectorXd CalcForceVector(const Eigen::VectorXd& p_des,
                                  const Eigen::VectorXd& p_actual,
                                  const Eigen::Vector3d& p_l,
                                  const Eigen::Vector3d& p_r,
                                  double psi_l, double psi_r) const;

  void CalcInput(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* u) const;
  double CalcFootYaw(const drake::multibody::MultibodyPlant<double>& plant,
                     const drake::systems::Context<double> &context,
                     const drake::multibody::Frame<double> &toe_frame) const;

//                drake::trajectories::Trajectory<double>* traj
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const double w_;

  // A list of pairs of contact body frame and contact point
//   const std::vector<
//       std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
//       feet_contact_points_;

  int state_port_desired_;
  int state_port_robot_;
  int radio_port_;
  int output_forces;

};

}  // namespace dairlib::cassie::zachary_standing_contoller