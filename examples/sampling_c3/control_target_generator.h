
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>
#include <numbers>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "dairlib/lcmt_radio_out.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "systems/framework/state_vector.h"

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/systems/framework/leaf_system.h"
inline constexpr double kPi = std::numbers::pi;

using drake::systems::BasicVector;
using drake::trajectories::PiecewiseQuaternionSlerp;

namespace dairlib {
namespace systems {

class TargetGenerator : public drake::systems::LeafSystem<double> {
 public:
  TargetGenerator(const drake::multibody::MultibodyPlant<double>& object_plant);

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_end_effector_target() const {
    return this->get_output_port(end_effector_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_object_target()
      const {
    return this->get_output_port(object_target_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_object_velocity_target() const {
    return this->get_output_port(object_velocity_target_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_object_final_target() const {
    return this->get_output_port(object_final_target_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_target_gen_info()
      const {
    return this->get_output_port(target_gen_info_port_);
  }

  void SetRemoteControlParameters(
      const int& goal_mode, const Eigen::VectorXd& target_object_position,
      const Eigen::VectorXd& target_object_orientation,
      const double& lookahead_step_size, const double& lookahead_angle,
      const double& angle_hysteresis, const double& angle_err_to_vel_factor,
      const double& ee_target_z_offset_above_object,
      const double& position_success_threshold,
      const double& orientation_success_threshold,
      const Eigen::VectorXd& random_goal_x_limits,
      const Eigen::VectorXd& random_goal_y_limits,
      const Eigen::VectorXd& random_goal_radius_limits,
      const double& resting_object_height);

 protected:
  // Purely virtual functions that have to be implemented in the derived
  // classes.
  virtual const std::vector<Eigen::Quaterniond>& GetNominalOrientations()
      const = 0;

  // These are the base class functions that will be used in the derived
  // classes.
  virtual void CalcEndEffectorTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  virtual void CalcObjectTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  virtual void CalcObjectVelocityTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  virtual void OutputObjectFinalTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  virtual void OutputTargetGeneratorInfo(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* target) const;

  virtual void SetRandomizedTargetFinalObjectPosition() const;
  virtual void SetRandomizedTargetFinalObjectOrientation() const;
  virtual void CycleThroughOrientationSequence() const;
  void OnGoalReached() const;
  std::pair<Eigen::Quaterniond, Eigen::Vector3d>
  GenerateLineTrajectoryWithLookahead(
      const Eigen::Quaterniond& quat_curr_orientation,
      const Eigen::Vector3d& obj_curr_position) const;

  mutable std::mt19937 rng_{std::random_device{}()};

  // Input ports
  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex object_state_port_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  drake::systems::OutputPortIndex object_target_port_;
  drake::systems::OutputPortIndex object_velocity_target_port_;
  drake::systems::OutputPortIndex object_final_target_port_;
  drake::systems::OutputPortIndex target_gen_info_port_;

  // Parameters for the target generator.
  mutable Eigen::VectorXd target_final_object_position_;
  mutable Eigen::VectorXd target_final_object_orientation_;
  double lookahead_step_size_;
  double lookahead_angle_;
  double angle_hysteresis_;
  double angle_err_to_vel_factor_;
  double ee_target_z_offset_above_object_;
  double position_success_threshold_;
  double orientation_success_threshold_;
  Eigen::VectorXd random_goal_x_limits_;
  Eigen::VectorXd random_goal_y_limits_;
  Eigen::VectorXd random_goal_radius_limits_;
  mutable Eigen::Vector3d last_rotation_axis_ = Eigen::Vector3d::Zero();
  double resting_object_height_;

  enum GoalMode {
    CHANGING_GOAL_RANDOM,
    CHANGING_GOAL_ORIENTATION_SEQUENCE,
    FIXED_GOAL
  };
  GoalMode goal_mode_;

  mutable int goal_counter_ = 1;
  mutable int orientation_index_ = -1;
};

// Derived jacktoy target generator class.
// This class will be used to generate targets for the jack toy.
// Nominal quaternions for the object.
inline const Eigen::Quaterniond kQUAT_ALL_UP{
    0.8804762392171493, 0.27984814233312133, -0.3647051996310009,
    -0.11591689595929514};
inline const Eigen::Quaterniond kQUAT_RED_DOWN{
    0.8804762392171495, 0.27984814233312133, 0.3647051996310008,
    0.11591689595929511};
inline const Eigen::Quaterniond kQUAT_BLUE_UP{
    0.7045563426109883, -0.06000300064686593, 0.45576803893928247,
    -0.540625096237162};
inline const Eigen::Quaterniond kQUAT_ALL_DOWN{
    0.45576803893928264, -0.5406250962371619, 0.7045563426109882,
    -0.060003000646866145};
inline const Eigen::Quaterniond kQUAT_GREEN_UP{
    0.36470519963100106, 0.11591689595929516, 0.8804762392171492,
    0.27984814233312133};
inline const Eigen::Quaterniond kQUAT_BLUE_DOWN{
    0.060003000646866235, 0.7045563426109882, 0.540625096237162,
    0.4557680389392827};
inline const Eigen::Quaterniond kQUAT_RED_UP{
    -0.2798481423331213, 0.8804762392171495, -0.11591689595929505,
    0.3647051996310012};
inline const Eigen::Quaterniond kQUAT_GREEN_DOWN{
    -0.8204732385702831, 0.42470820027786693, 0.1759198966061614,
    0.3398511429799875};

class TargetGeneratorJacktoy : public TargetGenerator {
 public:
  TargetGeneratorJacktoy(
      const drake::multibody::MultibodyPlant<double>& object_plant)
      : TargetGenerator(object_plant) {}

 protected:
  // Override the base class function to provide the valid orientations for the
  // jack toy.
  const std::vector<Eigen::Quaterniond>& GetNominalOrientations()
      const override {
    return nominal_orientations_;
  }

 private:
  // Nominal orientations for the jack to be balanced on the ground.
  const std::vector<Eigen::Quaterniond> nominal_orientations_{
      kQUAT_ALL_UP,   kQUAT_RED_DOWN,  kQUAT_BLUE_UP, kQUAT_ALL_DOWN,
      kQUAT_GREEN_UP, kQUAT_BLUE_DOWN, kQUAT_RED_UP,  kQUAT_GREEN_DOWN};
};

// push-t specific target generator class.
// Nominal quaternions for the object.
inline const Eigen::Quaterniond kQUAT_FLAT{1.0, 0.0, 0.0, 0.0};

class TargetGeneratorPushT : public TargetGenerator {
 public:
  TargetGeneratorPushT(
      const drake::multibody::MultibodyPlant<double>& object_plant)
      : TargetGenerator(object_plant) {}

 protected:
  // Override the base class function to provide the valid orientations for the
  // push-t.
  const std::vector<Eigen::Quaterniond>& GetNominalOrientations()
      const override {
    return nominal_orientations_;
  }

 private:
  // Nominal orientation for the T to be flat on the ground.
  const std::vector<Eigen::Quaterniond> nominal_orientations_{kQUAT_FLAT};
};

// Derived box topple target generator class.
// Nominal quaternions for the object.
inline const Eigen::Quaterniond kQUAT_1{0.0, 1.0, 0.0, 0.0};
inline const Eigen::Quaterniond kQUAT_2{0.7071, 0.7071, 0.0, 0.0};
inline const Eigen::Quaterniond kQUAT_3{0.7071, -0.7071, 0.0, 0.0};
inline const Eigen::Quaterniond kQUAT_4{0.7071, 0.0, 0.7071, 0.0};
inline const Eigen::Quaterniond kQUAT_5{0.7071, 0.0, -0.7071, 0.0};
inline const Eigen::Quaterniond kQUAT_6{1.0, 0.0, 0.0, 0.0};

class TargetGeneratorBoxTopple : public TargetGenerator {
 public:
  TargetGeneratorBoxTopple(
      const drake::multibody::MultibodyPlant<double>& object_plant)
      : TargetGenerator(object_plant) {}

 protected:
  // Override the base class function to provide the valid orientations for the
  // box topple.
  const std::vector<Eigen::Quaterniond>& GetNominalOrientations()
      const override {
    return nominal_orientations_;
  }

 private:
  // Nominal orientations for the box to be balanced on the ground.
  const std::vector<Eigen::Quaterniond> nominal_orientations_{
      kQUAT_1, kQUAT_2, kQUAT_3, kQUAT_4, kQUAT_5, kQUAT_6};
};

// ball rolling specific target generator class.
// Nominal quaternions for the object.
inline const Eigen::Quaterniond kQUAT_BALL{1.0, 0.0, 0.0, 0.0};
class TargetGeneratorBallRolling : public TargetGenerator {
 public:
  TargetGeneratorBallRolling(
      const drake::multibody::MultibodyPlant<double>& object_plant)
      : TargetGenerator(object_plant) {}

  void SetBallRollingParameters(
      const Eigen::VectorXd& target_final_object_orientation,
      const double& traj_radius, const double& x_c, const double& y_c,
      const double& lead_angle, const double& angle_hysteresis,
      const double& angle_err_to_vel_factor,
      const double& ee_target_z_offset_above_object,
      const double& resting_object_height);

 protected:
  // Override the base class function to provide the valid orientations for the
  // ball rolling.
  const std::vector<Eigen::Quaterniond>& GetNominalOrientations()
      const override {
    return nominal_orientations_;
  }

  virtual void CalcObjectTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const override;
  // Final object position is always same as target position.
  void OutputObjectFinalTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const override {
    // Reuse the target object position as the final object target since
    // the ball rolling task doesn't have a final object target.
    // Drake allows to read the already computed "object target vector".
    const auto& obj_target =
        this->get_output_port_object_target()
            .template Eval<drake::systems::BasicVector<double>>(context);

    // Copy that 7-element vector into the final-target port.
    target->SetFromVector(obj_target.get_value());
  }

 private:
  // Nominal orientation for the ball to be upright.
  const std::vector<Eigen::Quaterniond> nominal_orientations_{kQUAT_BALL};
  Eigen::VectorXd target_final_object_orientation_;
  double traj_radius_;
  double x_c_;
  double y_c_;
  double lead_angle_;
};

}  // namespace systems
}  // namespace dairlib
