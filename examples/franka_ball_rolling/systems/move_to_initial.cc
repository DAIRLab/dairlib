#include "move_to_initial.h"
#include <iostream>


using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using drake::systems::Context;
using drake::systems::State;
using drake::math::RotationMatrix;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;

namespace dairlib {
namespace systems {

MoveToInitial::MoveToInitial(
    const SimulateFrankaParams& sim_param,
    const HeuristicGaitParams& heuristic_param) {
    // INPUT PORTS
    // TODO:: make dimension not hardcoded, and future clean up for newer impedacne ports
    state_input_port_ =
            this->DeclareVectorInputPort(
                    "x, u, t",
                    OutputVector<double>(14, 13, 7))
            .get_index();
    // OUTPUT PORTS
    // TODO:: make dimension not hardcoded
    target_port_ =
            this->DeclareVectorOutputPort(
                    "xee, xball, xee_dot, xball_dot, lambda, visualization",
                    TimestampedVector<double>(38), &MoveToInitial::CalcTarget)
            .get_index();

    // Set Trajectory Patameters
    SetParameters(sim_param, heuristic_param);

    first_message_time_idx_ = this->DeclareAbstractState(
            drake::Value<double>(0));
    received_first_message_idx_ = this->DeclareAbstractState(
            drake::Value<bool>(false));

    this->DeclarePerStepUnrestrictedUpdateEvent(
            &MoveToInitial::UpdateFirstMessageTime);
}

EventStatus MoveToInitial::UpdateFirstMessageTime(const Context<double>& context,
                                                  State<double>* state) const {
    auto& received_first_message = state->get_mutable_abstract_state<bool>(received_first_message_idx_);
    auto& first_message_time = state->get_mutable_abstract_state<double>(first_message_time_idx_);

    if (!received_first_message){
        auto robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
        double timestamp = robot_output->get_timestamp();
        received_first_message = true;
        first_message_time = timestamp;
        return EventStatus::Succeeded();
    }
    return EventStatus::Succeeded();
}

void MoveToInitial::SetParameters(const SimulateFrankaParams& sim_param,
                                              const HeuristicGaitParams heuristic_param) {
  // Set parameters
  stabilize_time1_ = heuristic_param.stabilize_time1;
  move_time_ = heuristic_param.move_time;

  initial_start_ = heuristic_param.initial_start;
  initial_finish_ = heuristic_param.initial_finish;
  x_c_= sim_param.x_c;
  y_c_= sim_param.y_c;
  traj_radius_ = sim_param.traj_radius;
  initial_phase_ = sim_param.phase;
  object_height_ = sim_param.ball_radius + sim_param.ground_offset_frame(2);

  tilt_degrees_ = heuristic_param.tilt_degrees;
}

void MoveToInitial::CalcTarget(
    const Context<double>& context,
    TimestampedVector<double>* output) const {

    auto robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
    double timestamp = robot_output->get_timestamp();

    auto first_message_time = context.get_abstract_state<double>(first_message_time_idx_);
    double curr_time = timestamp - first_message_time;

    Vector3d start = initial_start_;
    Vector3d finish = initial_finish_;
    Quaterniond start_orientation(0, 1, 0, 0);

    finish(0) = x_c_ + traj_radius_ * sin(initial_phase_ * PI/ 180) + finish(0);
    finish(1) = y_c_ + traj_radius_ * cos(initial_phase_ * PI / 180) + finish(1);
    std::vector<Vector3d> target = move_to_initial_position(start, finish, curr_time,stabilize_time1_,
                                                                   move_time_);
    Quaterniond orientation_d = move_to_initial_orientation(start_orientation, tilt_degrees_, curr_time,
                                                            stabilize_time1_, move_time_);

    // linear interpolate orientation of the end-effector

    VectorXd st_desired = VectorXd::Zero(38);
    st_desired.head(3) << target[0];
    st_desired.segment(3, 4) << orientation_d.w(), orientation_d.x(), orientation_d.y(), orientation_d.z();
    st_desired.segment(11, 3) << finish(0), finish(1), object_height_;
    st_desired.segment(14, 3) << target[1];
    st_desired.segment(32, 3) << finish(0), finish(1), object_height_;
    st_desired.tail(3) << finish(0), finish(1), object_height_;

    output->SetDataVector(st_desired);
    output->set_timestamp(timestamp);
}

std::vector<Vector3d> MoveToInitial::move_to_initial_position(
    const Vector3d& start,
    const Vector3d& finish,
    double curr_time, double stabilize_time,
    double move_time)const{

  Eigen::Vector3d zero(0,0,0);

  if (curr_time < stabilize_time){
    return {start, zero};
  }
  else if (curr_time < stabilize_time + move_time){
    double a = (curr_time-stabilize_time) / (move_time);
    Eigen::Vector3d p = (1-a)*start + a*finish;
    Eigen::Vector3d v = (finish - start) / move_time;
    return {p, v};
  }
  else{
    return {finish, zero};
  }
}

Quaterniond MoveToInitial::move_to_initial_orientation(
        const Quaterniond& start_orientation,
        double tilt_degrees,
        double curr_time, double stabilize_time,
        double move_time) const{

    RotationMatrix<double> R_start(start_orientation);
    double duration = stabilize_time + move_time;
    if (curr_time < stabilize_time + move_time){
        RotationMatrix<double> rot_y = RotationMatrix<double>::MakeYRotation(
                (curr_time / duration) * tilt_degrees * PI / 180);
        Quaterniond orientation_d = (R_start * rot_y).ToQuaternion();
        return orientation_d;
    }
    else{
        RotationMatrix<double> rot_y = RotationMatrix<double>::MakeYRotation(
                tilt_degrees * PI / 180);
        Quaterniond orientation_d = (R_start * rot_y).ToQuaternion();
        return orientation_d;
    }
}
}  // namespace systems
}  // namespace dairlib