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
inline std::vector<std::vector<Eigen::Quaterniond>> MakeNominalOrientationsPlanar(int n) {
  std::vector<std::vector<Eigen::Quaterniond>> orientation_storage;

  for (int i = 0; i < n; ++i) {
    orientation_storage.push_back({kQUAT_FLAT});
  }

  return orientation_storage;
}


namespace dairlib {
namespace systems {

class SamplingC3GoalGenerator : public drake::systems::LeafSystem<double> {
 public:
  SamplingC3GoalGenerator(
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params,
    std::vector<std::vector<Eigen::Quaterniond>> nominal_orientations,
    std::vector<drake::multibody::ModelInstanceIndex> object_indices);

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

  std::vector<const drake::systems::InputPort<double>*> 
    get_input_ports_object_state() const {
    std::vector<const drake::systems::InputPort<double>*> output;
    for (drake::systems::InputPortIndex port : object_state_ports_) {
      output.push_back(
        &this->get_input_port(port)
      );
    } 
    return output;
  }

  const drake::systems::OutputPort<double>&
  get_output_port_end_effector_target() const {
    return this->get_output_port(end_effector_target_port_);
  }

  std::vector<const drake::systems::OutputPort<double>*> 
    get_output_ports_object_target() const {
    std::vector<const drake::systems::OutputPort<double>*> output;
    for (drake::systems::OutputPortIndex port : object_target_ports_) {
      output.push_back(
        &this->get_output_port(port)
      );
    } 
    return output;
  }

  std::vector<const drake::systems::OutputPort<double>*>
    get_output_ports_object_velocity_target() const {
    std::vector<const drake::systems::OutputPort<double>*> output;
    for (drake::systems::OutputPortIndex port : object_velocity_target_ports_) {
      output.push_back(
        &this->get_output_port(port)
      );
    } 
    return output;
  }

  std::vector<const drake::systems::OutputPort<double>*>
  get_output_ports_object_final_target() const {
    std::vector<const drake::systems::OutputPort<double>*> output;
    for (drake::systems::OutputPortIndex port : object_final_target_ports_) {
      output.push_back(
        &this->get_output_port(port)
      );
    } 
    return output;
  }

  const drake::systems::OutputPort<double>& get_output_port_target_gen_info()
      const {
    return this->get_output_port(target_gen_info_port_);
  }




private:
  std::vector<drake::multibody::ModelInstanceIndex> object_indices_;
  mutable std::vector<bool> reached_goal_;

  const std::vector<Eigen::Quaterniond>& GetNominalOrientations(int index) const {
    return nominal_orientations_.at(index); }

  void CalcEndEffectorTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target) const;
  void CalcObjectTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target,
      int index) const;
  void CalcObjectVelocityTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target,
      int index) const;
  void OutputObjectFinalTarget(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* target,
      int index) const;

  void OutputGoalGeneratorInfo(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* target) const;
  void SetRandomizedTargetFinalObjectPosition(int index) const;
  void SetRandomizedTargetFinalObjectOrientation(int index) const;
  void AssignObjectIndexToGoalSamplingArea() const;
  void CycleThroughOrientationSequence(int index) const;
  void OnGoalReached(int index) const;
  std::pair<Eigen::Quaterniond, Eigen::Vector3d>
    GenerateLineTrajectoryWithLookahead(
      const Eigen::Quaterniond& quat_curr_orientation,
      const Eigen::Vector3d& obj_curr_position,
      int index) const;

  // Input ports
  drake::systems::InputPortIndex radio_port_;
  std::vector<drake::systems::InputPortIndex> object_state_ports_;
  drake::systems::OutputPortIndex end_effector_target_port_;
  std::vector<drake::systems::OutputPortIndex> object_target_ports_;
  std::vector<drake::systems::OutputPortIndex> object_velocity_target_ports_;
  std::vector<drake::systems::OutputPortIndex> object_final_target_ports_;
  drake::systems::OutputPortIndex target_gen_info_port_;

  const SamplingC3GoalParams goal_params_;
  std::vector<std::vector<Eigen::Quaterniond>> nominal_orientations_;
  mutable std::vector<Eigen::Vector3d> target_final_object_positions_;
  mutable std::vector<Eigen::Vector4d> target_final_object_orientations_;
  
  // indicates target final position of which object is in which sampling area
  mutable std::vector<int> object_index_to_sampling_area_index_map_;
  mutable Eigen::Vector3d datum_position_ = Eigen::VectorXd::Constant(3, std::numeric_limits<double>::quiet_NaN());

  mutable Eigen::Vector3d last_rotation_axis_ = Eigen::Vector3d::Zero();
  mutable int goal_counter_ = 1;
  mutable int orientation_index_ = -1;
};

class SamplingC3GoalGeneratorJacktoy : public SamplingC3GoalGenerator {
 public:
  SamplingC3GoalGeneratorJacktoy(
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params, 
    std::vector<drake::multibody::ModelInstanceIndex> object_indices
  ) :
      SamplingC3GoalGenerator(
        object_plant,
        goal_params,
        std::vector<std::vector<Eigen::Quaterniond>>{
            kNominalOrientationsJack},
        object_indices) {}
};


class SamplingC3GoalGeneratorPlanar : public SamplingC3GoalGenerator {
 public:
  SamplingC3GoalGeneratorPlanar(
    const drake::multibody::MultibodyPlant<double>& object_plant,
    const SamplingC3GoalParams& goal_params,
    std::vector<drake::multibody::ModelInstanceIndex> object_indices
  ) :
      SamplingC3GoalGenerator(
        object_plant, goal_params, MakeNominalOrientationsPlanar(object_indices.size()), object_indices) {}

 private:
  mutable int orientation_index_ = 0;
};


}  // namespace systems
}  // namespace dairlib
