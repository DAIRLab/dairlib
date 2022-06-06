#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/framework/output_vector.h"


namespace dairlib {
namespace systems {

 class AlipFootstepPlanner: public drake::systems::LeafSystem<double> {
 public:
  AlipFootstepPlanner(const drake::multibody::MultibodyPlant<double>& plant,
                      drake::systems::Context<double>* context,
                      std::vector<int> left_right_support_fsm_states,
                      std::vector<double> left_right_support_durations,
                      std::vector<std::pair<const Eigen::Vector3d,
                      const drake::multibody::Frame<double>&>> left_right_foot,
                      double double_support_duration,
                      double max_com_to_footstep_dist, double footstep_offset,
                      double center_line_offset);

  const drake::systems::InputPort<double>& get_input_port_alip_state() const {
    return this->get_input_port(alip_state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_vdes() const {
    return this->get_input_port(vdes_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
   const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
   const {
     return this->get_input_port(liftoff_time_port_);
   }

 private:
   void CalcFootStepAndStanceFootHeight(
       const drake::systems::Context<double>& context,
       const OutputVector<double>* robot_output,
       const double end_time_of_this_interval, Eigen::Vector2d* x_fs,
       double* stance_foot_height) const;

   void CalcTarget(const drake::systems::Context<double>& context,
       BasicVector<double>* target) const;

   int state_port_;
   int fsm_port_;
   int alip_state_port_;
   int vdes_port_;
   int liftoff_time_port_;

   const drake::multibody::MultibodyPlant<double>& plant_;
   drake::systems::Context<double>* context_;
   const drake::multibody::BodyFrame<double>& world_;

   std::vector<int> left_right_support_fsm_states_;
   double double_support_duration_;

   // Parameters
   double m_;
   double max_com_to_footstep_dist_;
   const double footstep_offset_;     // in meters
   const double center_line_offset_;  // in meters

   // Maps
   std::map<int, std::pair<const Eigen::Vector3d,
                           const drake::multibody::Frame<double>&>>
       stance_foot_map_;
   std::map<int, double> duration_map_;
};

}
}
