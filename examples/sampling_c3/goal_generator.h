#include <vector>
#include <numbers>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "common/math_utils.h"
#include "dairlib/lcmt_radio_out.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "systems/framework/state_vector.h"
#include "examples/sampling_c3/parameter_headers/goal_params.h"

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

using drake::systems::BasicVector;
using drake::trajectories::PiecewiseQuaternionSlerp;


// Define the nominal orientations for the jack demo.
inline const Eigen::Quaterniond kQuatAllUp{
  0.88047623921714, 0.279848142333121, -0.36470519963100, -0.115916895959295};
inline const Eigen::Quaterniond kQuatRedDown{
  0.88047623921714, 0.279848142333121, 0.36470519963100, 0.115916895959295};
inline const Eigen::Quaterniond kQuatBlueUp{
  0.70455634261098, -0.060003000646865, 0.455768038939282, -0.5406250962371};
inline const Eigen::Quaterniond kQuatAllDown{
  0.455768038939282, -0.54062509623716, 0.70455634261098, -0.0600030006468661};
inline const Eigen::Quaterniond kQuatGreenUp{
  0.364705199631001, 0.115916895959295, 0.88047623921714, 0.279848142333121};
inline const Eigen::Quaterniond kQuatBlueDown{
  0.0600030006468662, 0.70455634261098, 0.5406250962371, 0.45576803893928};
inline const Eigen::Quaterniond kQuatRedUp{
  -0.27984814233312, 0.88047623921714, -0.115916895959295, 0.36470519963100};
inline const Eigen::Quaterniond kQuatGreenDown{
  -0.82047323857028, 0.424708200277866, 0.17591989660616, 0.33985114297998};
inline const std::vector<Eigen::Quaterniond> kNominalOrientationsJack{
  kQuatAllUp,   kQuatRedDown,  kQuatBlueUp, kQuatAllDown,
  kQuatGreenUp, kQuatBlueDown, kQuatRedUp,  kQuatGreenDown};

// Define the nominal orientations for any planar demo.
inline const Eigen::Quaterniond kQUAT_FLAT{1.0, 0.0, 0.0, 0.0};
inline const std::vector<Eigen::Quaterniond> kNominalOrientationsPlanar{
  kQUAT_FLAT};


namespace dairlib {
namespace systems {

class SamplingC3GoalGenerator : public drake::systems::LeafSystem<double> {
 public:
  SamplingC3GoalGenerator(
    const int end_effector_num_positions,
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params,
    const std::vector<Eigen::Quaterniond>& nominal_orientations);

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

 private:
  const std::vector<Eigen::Quaterniond>& GetNominalOrientations() const {
    return nominal_orientations_; }

  void CalcEndEffectorTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  void CalcObjectTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  void CalcObjectVelocityTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  void OutputObjectFinalTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  void OutputGoalGeneratorInfo(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* target) const;
  void SetRandomizedTargetFinalObjectPosition() const;
  void SetRandomizedTargetFinalObjectOrientation() const;
  void CycleThroughOrientationSequence() const;
  void OnGoalReached() const;
  std::pair<Eigen::Quaterniond, Eigen::Vector3d>
    GenerateLineTrajectoryWithLookahead(
      const Eigen::Quaterniond& quat_curr_orientation,
      const Eigen::Vector3d& obj_curr_position) const;

  // Input ports
  drake::systems::InputPortIndex radio_port_;
  drake::systems::InputPortIndex object_state_port_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  drake::systems::OutputPortIndex object_target_port_;
  drake::systems::OutputPortIndex object_velocity_target_port_;
  drake::systems::OutputPortIndex object_final_target_port_;
  drake::systems::OutputPortIndex target_gen_info_port_;

  const SamplingC3GoalParams goal_params_;
  const std::vector<Eigen::Quaterniond> nominal_orientations_;
  mutable Eigen::VectorXd target_final_object_position_;
  mutable Eigen::VectorXd target_final_object_orientation_;
  mutable Eigen::Vector3d last_rotation_axis_ = Eigen::Vector3d::Zero();
  mutable int goal_counter_ = 1;
  mutable int orientation_index_ = -1;
  const int end_effector_num_positions_;
};

class SamplingC3GoalGeneratorJacktoy : public SamplingC3GoalGenerator {
 public:
  SamplingC3GoalGeneratorJacktoy(
    const int end_effector_num_positions,
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params) :
      SamplingC3GoalGenerator(
        end_effector_num_positions, object_plant, goal_params, kNominalOrientationsJack) {}
};


class SamplingC3GoalGeneratorPlanar : public SamplingC3GoalGenerator {
 public:
  SamplingC3GoalGeneratorPlanar(
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params) :
      SamplingC3GoalGenerator(
        end_effector_num_positions, object_plant, goal_params, kNominalOrientationsPlanar) {}
};


class SamplingC3GoalGeneratorTrifinger : public SamplingC3GoalGenerator {
 public:
  SamplingC3GoalGeneratorTrifinger(
    const int end_effector_num_positions,
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params) :
      SamplingC3GoalGenerator(
        end_effector_num_positions, object_plant, goal_params, kNominalOrientationsPlanar) {}
};

}  // namespace systems
}  // namespace dairlib
