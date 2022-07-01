#pragma once

#include "multibody/kinematic/kinematic_evaluator_set.h"
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
      drake::systems::Context<double>* context,
      const multibody::KinematicEvaluatorSet<double>* kinematic_evaluators,
      const Eigen::MatrixXd& K_p, const Eigen::MatrixXd& K_d, double k_cp);

  const drake::systems::InputPort<double>& get_input_port_state_desired() const {
    return this->get_input_port(state_port_desired_);
  }

  const drake::systems::InputPort<double>& get_input_port_state_actual() const{
      return this->get_input_port(state_port_robot_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_torques() const{
      return this->get_output_port(commanded_torque_port_);
  }

 private:
  Eigen::VectorXd CalcForceVector(const Eigen::VectorXd& p_des,
                                  const Eigen::VectorXd& p_actual,
                                  const Eigen::Vector3d& p_l,
                                  const Eigen::Vector3d& p_r,
                                  double psi_l, double psi_r) const;

  void CalcInput(const drake::systems::Context<double>& context,
                 systems::TimestampedVector<double>* u) const;

  double CalcFootYaw(const drake::multibody::MultibodyPlant<double>& plant,
                     const drake::systems::Context<double> &context,
                     const drake::multibody::Frame<double> &toe_frame) const;


  // plant parameters
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const multibody::KinematicEvaluatorSet<double>* kinematic_evaluators_;
  std::vector<std::string> spring_vel_names_ = {"knee_joint_leftdot",
                                                "ankle_spring_joint_leftdot",
                                                "knee_joint_rightdot",
                                                "ankle_spring_joint_rightdot",};
  std::map<std::string, int> name_to_vel_map_;
  const double w_;
  const double y_h_ = 0.135; // y-axis distance from pelvis center to hip roll

  // Gains
  const Eigen::MatrixXd& K_p_;
  const Eigen::MatrixXd& K_d_;
  double k_cp_;

  // A list of pairs of contact body frame and contact point
  //   const std::vector<
  //       std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
  //       feet_contact_points_;



  int state_port_desired_;
  int state_port_robot_;
  int radio_port_;
  int commanded_torque_port_;

};

}  // namespace dairlib::cassie::zachary_standing_contoller