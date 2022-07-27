//
// Created by brian on 7/26/22.
//

// dairlib includes
#include "alip_s2s_planner.h"
#include "convex_foothold.h"
#include "systems/filters/s2s_kalman_filter.h"

// drake includes
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace dairlib::systems::controllers {

using alip_utils::PointOnFramed;
using alip_utils::CalcA;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector4d;
using Eigen::Vector3d;

using drake::math::RotationMatrixd;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::State;

AlipS2SPlanner::AlipS2SPlanner(const MultibodyPlant<double>& plant,
                               Context<double>* context,
                               std::vector<int> left_right_stance_fsm_states,
                               std::vector<double> left_right_stance_durations,
                               std::vector<PointOnFramed> left_right_foot,
                               AlipS2SControllerParameters params) :
                               plant_(plant),
                               context_(context),
                               world_(plant.world_frame()),
                               m_(plant.CalcTotalMass(*context)),
                               left_right_stance_fsm_states_(left_right_stance_fsm_states),
                               params_(params) {

  DRAKE_DEMAND(left_right_stance_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_stance_durations.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);

  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
  nu_ = plant_.num_actuators();

  // Declare discrete states first (otherwise ports aren't assigned properly)
  fsm_state_idx_ = this->DeclareDiscreteState(1);
  time_to_impact_idx_ = this->DeclareDiscreteState(1);
  time_since_impact_idx_ = this->DeclareDiscreteState(1);
  footstep_target_idx_ = this->DeclareDiscreteState(3);
  zdot_com_pre_impact_idx_ = this->DeclareDiscreteState(1);

  // Then the abstract states why not
  if (params.filter_alip_state) {
    MatrixXd A = CalcA(params_.z_com_des, m_);
    MatrixXd B = -MatrixXd::Identity(4,2);
    MatrixXd C = MatrixXd::Identity(4,4);
    MatrixXd G = MatrixXd::Identity(4,4);

    S2SKalmanFilterData filter_data =
        {{A, B, C, params_.Q_filt, params_.R_filt}, G};
    S2SKalmanFilter filter = S2SKalmanFilter(filter_data);
    std::pair<S2SKalmanFilter,
              S2SKalmanFilterData> model_filter = {filter, filter_data};
    s2s_filter_idx_ = this->DeclareAbstractState(
        drake::Value<std::pair<S2SKalmanFilter,
                               S2SKalmanFilterData>>(model_filter));
  }

  /// TODO (@Brian-Acosta) For now we will reconstruct the mathematical
  /// program every loop. In the future, we should recycle the
  /// mathematical program, decision variables, etc.
  std::vector<drake::solvers::VectorXDecisionVariable> xx;
  std::vector<drake::solvers::VectorXDecisionVariable> uu;
  std::pair<drake::solvers::VectorXDecisionVariable,
            drake::solvers::VectorXDecisionVariable> model_mpc_sol;
  planar_alip_mpc_sol_idx_ = this->DeclareAbstractState(
      drake::Value<
          std::pair<drake::solvers::VectorXDecisionVariable,
                    drake::solvers::VectorXDecisionVariable>>(model_mpc_sol));
  z_com_mpc_sol_idx_ = this->DeclareAbstractState(
      drake::Value<
          std::pair<drake::solvers::VectorXDecisionVariable,
                    drake::solvers::VectorXDecisionVariable>>(model_mpc_sol));

  // Input ports
  state_input_port_ = this->DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(nq_, nv_, nu_)).get_index();
  vdes_input_port_ = this->DeclareVectorInputPort(
      "xy walking speed", BasicVector<double>(2)).get_index();
  ConvexFoothold model_foothold(MatrixXd::Zero(0,0), VectorXd::Zero(0, 0),
                                MatrixXd::Zero(0, 0), VectorXd::Zero(0, 0));
  foothold_input_port_ = this->DeclareAbstractInputPort(
      "next stepping stone", drake::Value<ConvexFoothold>(model_foothold))
          .get_index();

  // Direct state output ports
  fsm_output_port_ = this->DeclareStateOutputPort(
      "fsm", fsm_state_idx_).get_index();
  time_to_impact_output_port_ = this->DeclareStateOutputPort(
      "seconds until next imapct", time_to_impact_idx_).get_index();
  time_since_impact_output_port_ = this->DeclareStateOutputPort(
      "seconds since last impact", time_since_impact_idx_).get_index();
  footstep_target_output_port_ = this->DeclareStateOutputPort(
      "footstep target", footstep_target_idx_).get_index();


  this->DeclarePerStepUnrestrictedUpdateEvent(
      &AlipS2SPlanner::UnrestrictedUpdate);

  // Construct maps
  duration_map_.insert({left_right_stance_fsm_states.at(0),
                        left_right_stance_durations.at(0)});
  duration_map_.insert({left_right_stance_fsm_states.at(1),
                        left_right_stance_durations.at(1)});
  stance_foot_map_.insert(
      {left_right_stance_fsm_states.at(0), left_right_foot.at(0)});
  stance_foot_map_.insert(
      {left_right_stance_fsm_states.at(1), left_right_foot.at(1)});

}

EventStatus AlipS2SPlanner::UnrestrictedUpdate(const Context<double>& context,
                                               State<double>* state) const {
  return EventStatus::Succeeded();
}

}