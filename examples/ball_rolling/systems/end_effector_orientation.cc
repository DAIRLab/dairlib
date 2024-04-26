#include "end_effector_orientation.h"

#include <iostream>

#include "dairlib/lcmt_radio_out.hpp"
#include "multibody/multibody_utils.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::string;

using dairlib::systems::OutputVector;
using drake::multibody::BodyFrame;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;

namespace dairlib {

EndEffectorOrientationGenerator::EndEffectorOrientationGenerator(
    const MultibodyPlant<double>& plant, Context<double>* context)
    : plant_(plant), context_(context), world_(plant.world_frame()) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x", OutputVector<double>(plant_.num_positions(),
                                                  plant_.num_velocities(),
                                                  plant_.num_actuators()))
                    .get_index();
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>();

  trajectory_port_ =
      this->DeclareAbstractInputPort(
              "trajectory",
              drake::Value<drake::trajectories::Trajectory<double>>(pp))
          .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  PiecewiseQuaternionSlerp<double> empty_slerp_traj;
  Trajectory<double>& traj_inst = empty_slerp_traj;
  this->DeclareAbstractOutputPort("end_effector_orientation", traj_inst,
                                  &EndEffectorOrientationGenerator::CalcTraj)
      .get_index();
}

PiecewiseQuaternionSlerp<double> EndEffectorOrientationGenerator::GeneratePose(
    const drake::systems::Context<double>& context) const {
  Eigen::VectorXd neutral_quaternion = VectorXd::Zero(4);
  neutral_quaternion(0) = 1;
  return drake::trajectories::PiecewiseQuaternionSlerp<double>(
      {0, 1}, {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Quaterniond(1, 0, 0, 0)});
}

void EndEffectorOrientationGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  //  // Read in finite state machine

  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(traj);
  if (radio_out->channel[14] and track_orientation_) {
    const auto& trajectory_input =
        this->EvalAbstractInput(context, trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    *casted_traj = *(PiecewiseQuaternionSlerp<double>*)dynamic_cast<
        const PiecewiseQuaternionSlerp<double>*>(&trajectory_input);
  } else {
    *casted_traj = GeneratePose(context);
  }
}

}  // namespace dairlib
