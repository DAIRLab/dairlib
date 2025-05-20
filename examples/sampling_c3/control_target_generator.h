#pragma once

#include <vector>
#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/state_vector.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "dairlib/lcmt_radio_out.hpp"
#include "drake/common/trajectories/piecewise_quaternion.h"


#define PI 3.14159265359

using drake::systems::BasicVector;
using drake::trajectories::PiecewiseQuaternionSlerp;

namespace dairlib {
namespace systems {

class TargetGenerator : public drake::systems::LeafSystem<double> {
    public:
        TargetGenerator(
            const drake::multibody::MultibodyPlant<double>& object_plant);
        
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

        const drake::systems::OutputPort<double>& get_output_port_object_velocity_target()
        const {
            return this->get_output_port(object_velocity_target_port_);
        }

        const drake::systems::OutputPort<double>& get_output_port_object_final_target()
        const {
            return this->get_output_port(object_final_target_port_);
        }

        const drake::systems::OutputPort<double>& get_output_port_target_gen_info()
        const {
            return this->get_output_port(target_gen_info_port_);
        }

        void SetRemoteControlParameters(
            const int& trajectory_type,
            const bool& use_changing_final_goal,
            const int& changing_final_goal_type,
            const bool& prevent_three_topples_for_random_goal_gen,
            const double& traj_radius,
            const double& x_c,
            const double& y_c,
            const double& lead_angle,
            const Eigen::VectorXd& target_object_position,
            const Eigen::VectorXd& target_object_orientation,
            const double& step_size,
            const double& start_point_x,
            const double& start_point_y,
            const double& end_point_x,
            const double& end_point_y,
            const double& lookahead_step_size,
            const double& lookahead_angle,
            const double& angle_hysteresis,
            const double& angle_err_to_vel_factor,
            const double& max_step_size,
            const double& ee_goal_height,
            const double& object_half_width,
            const double& position_success_threshold,
            const double& orientation_success_threshold,
            const Eigen::VectorXd& random_goal_x_limits,
            const Eigen::VectorXd& random_goal_y_limits,
            const Eigen::VectorXd& random_goal_radius_limits,
            const double& resting_object_height);

        protected:
            // Purely virtual functions that have to be implemented in the derived classes.
            virtual const std::vector<Eigen::Quaterniond>& ValidOrientations() const = 0;

            // These are the base class functions that will be used in the derived classes.
            virtual void CalcEndEffectorTarget(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* target) const;
            virtual void CalcObjectTarget(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* target) const;
            virtual void CalcObjectVelocityTarget(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* target) const;
            virtual void OutputObjectFinalTarget(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* target) const;
            virtual void OutputTargetGeneratorInfo(const drake::systems::Context<double>& context,
                                dairlib::lcmt_timestamped_saved_traj* target) const;
            
            virtual void SetRandomizedTargetFinalObjectPosition() const;
            virtual void SetRandomizedTargetFinalObjectOrientation() const;   
            
            // virtual functions with default no-op implementations
            virtual void CycleThroughOrientationSequence() const {}
            virtual bool three_topples_required(int) const { return false; }    
        
            drake::systems::InputPortIndex radio_port_;
            drake::systems::InputPortIndex object_state_port_;
            drake::systems::OutputPortIndex end_effector_target_port_;
            drake::systems::OutputPortIndex object_target_port_;
            drake::systems::OutputPortIndex object_velocity_target_port_;
            drake::systems::OutputPortIndex object_final_target_port_;
            drake::systems::OutputPortIndex target_gen_info_port_;
      
            int trajectory_type_;
            bool use_changing_final_goal_;
            bool prevent_three_topples_;
            double traj_radius_;
            double x_c_;
            double y_c_;
            double lead_angle_;
            mutable Eigen::VectorXd target_final_object_position_;
            mutable Eigen::VectorXd target_final_object_orientation_;
            double step_size_;
            double start_point_x_;
            double start_point_y_;
            double end_point_x_;
            double end_point_y_;
            double lookahead_step_size_;
            double lookahead_angle_;
            double angle_hysteresis_;
            double angle_err_to_vel_factor_;
            double max_step_size_;
            double ee_goal_height_;
            double object_half_width_;
            double position_success_threshold_;
            double orientation_success_threshold_;
            Eigen::VectorXd random_goal_x_limits_;
            Eigen::VectorXd random_goal_y_limits_;
            Eigen::VectorXd random_goal_radius_limits_;
            mutable Eigen::Vector3d last_rotation_axis_ = Eigen::Vector3d::Zero();
            double resting_object_height_;
        
            enum ChangingGoalType {CHANGING_GOAL_RANDOM,
                                CHANGING_GOAL_ORIENTATION_SEQUENCE};
            ChangingGoalType changing_goal_type_;
        
            mutable int goal_counter_ = 1;
            mutable int orientation_index_ = -1;
};

// Derived jacktoy target generator class.
// Here we will add additional functions such as prevent three topples, 
// cycle through orientation sequence, etc.
// This class will be used to generate targets for the jack toy.
// Nominal quaternions for the object.
#define QUAT_ALL_UP Eigen::Quaterniond(0.8804762392171493, 0.27984814233312133, -0.3647051996310009, -0.11591689595929514)
#define QUAT_RED_DOWN Eigen::Quaterniond(0.8804762392171495, 0.27984814233312133, 0.3647051996310008, 0.11591689595929511)
#define QUAT_BLUE_UP Eigen::Quaterniond(0.7045563426109883, -0.06000300064686593, 0.45576803893928247, -0.540625096237162)
#define QUAT_ALL_DOWN Eigen::Quaterniond(0.45576803893928264, -0.5406250962371619, 0.7045563426109882, -0.060003000646866145)
#define QUAT_GREEN_UP Eigen::Quaterniond(0.36470519963100106, 0.11591689595929516, 0.8804762392171492, 0.27984814233312133)
#define QUAT_BLUE_DOWN Eigen::Quaterniond(0.060003000646866235, 0.7045563426109882, 0.540625096237162, 0.4557680389392827)
#define QUAT_RED_UP Eigen::Quaterniond(-0.2798481423331213, 0.8804762392171495, -0.11591689595929505, 0.3647051996310012)
#define QUAT_GREEN_DOWN Eigen::Quaterniond(-0.8204732385702831, 0.42470820027786693, 0.1759198966061614, 0.3398511429799875)

class TargetGeneratorJacktoy : public TargetGenerator {
    public:
        TargetGeneratorJacktoy(
            const drake::multibody::MultibodyPlant<double>& object_plant) : 
            TargetGenerator(object_plant){ }
        
    
    protected:
        // Override the base class function to provide the valid orientations for the jack toy.
        bool three_topples_required(const int new_orientation_index) const override;
        void CycleThroughOrientationSequence() const override;
        const std::vector<Eigen::Quaterniond>& ValidOrientations() const override {
            return valid_orientations_;
        }

    private:

    // Nominal orientations for the jack to be balanced on the ground.
    const std::vector<Eigen::Quaterniond> valid_orientations_{
        QUAT_ALL_UP, QUAT_RED_DOWN, QUAT_BLUE_UP, QUAT_ALL_DOWN,
        QUAT_GREEN_UP, QUAT_BLUE_DOWN, QUAT_RED_UP, QUAT_GREEN_DOWN
    };
};

// push-t specific target generator class.
// Nominal quaternions for the object.
#define QUAT_FLAT Eigen::Quaterniond(1.0,0.0,0.0,0.0)

class TargetGeneratorPushT : public TargetGenerator {
    public:
        TargetGeneratorPushT(
            const drake::multibody::MultibodyPlant<double>& object_plant) : 
            TargetGenerator(object_plant){ }
        
    protected:
        // Override the base class function to provide the valid orientations for the push-t.
        const std::vector<Eigen::Quaterniond>& ValidOrientations() const override {
            return valid_orientations_;
        }

    private:
        // Nominal orientation for the T to be flat on the ground.
        const std::vector<Eigen::Quaterniond> valid_orientations_{QUAT_FLAT};
};

// Derived box topple target generator class.

// Nominal quaternions for the object.
// TODO: Rename these quaternions to be more accurate.
#define QUAT_1 Eigen::Quaterniond(0.0,1.0,0.0,0.0)
#define QUAT_2 Eigen::Quaterniond(0.7071,0.7071,0.0,0.0)
#define QUAT_3 Eigen::Quaterniond(0.7071,-0.7071,0.0,0.0)
#define QUAT_4 Eigen::Quaterniond(0.7071,0.0,0.7071,0.0)
#define QUAT_5 Eigen::Quaterniond(0.7071,0.0,-0.7071,0.0)
#define QUAT_6 Eigen::Quaterniond(1.0,0.0,0.0,0.0)

class TargetGeneratorBoxTopple : public TargetGenerator {
    public:
        TargetGeneratorBoxTopple(
            const drake::multibody::MultibodyPlant<double>& object_plant) : 
            TargetGenerator(object_plant){ }
        
    protected:
        // Override the base class function to provide the valid orientations for the box topple.
        const std::vector<Eigen::Quaterniond>& ValidOrientations() const override {
            return valid_orientations_;
        }

    private:
        // Nominal orientations for the box to be balanced on the ground.
        const std::vector<Eigen::Quaterniond> valid_orientations_{
            QUAT_1, QUAT_2, QUAT_3, QUAT_4,
            QUAT_5, QUAT_6
        };
};

// ball rolling specific target generator class.
// Nominal quaternions for the object.
#define QUAT_BALL Eigen::Quaterniond(1.0,0.0,0.0,0.0)
class TargetGeneratorBallRolling : public TargetGenerator {
    public:
        TargetGeneratorBallRolling(
            const drake::multibody::MultibodyPlant<double>& object_plant) : 
            TargetGenerator(object_plant){ }
        
    protected:
        // Override the base class function to provide the valid orientations for the ball rolling.
        const std::vector<Eigen::Quaterniond>& ValidOrientations() const override {
            return valid_orientations_;
        }

        // Final object position is always same as target position.
        void OutputObjectFinalTarget(const drake::systems::Context<double>& context,
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
        const std::vector<Eigen::Quaterniond> valid_orientations_{QUAT_BALL};
};

}  // namespace systems
}  // namespace dairlib

    