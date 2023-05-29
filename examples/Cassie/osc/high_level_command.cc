#include "examples/Cassie/osc/high_level_command.h"

#include <math.h>

#include <string>

#include "dairlib/lcmt_cassie_out.hpp"
#include "multibody/multibody_utils.h"

#include "drake/math/quaternion.h"
#include "drake/math/saturate.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Quaterniond;

using dairlib::systems::OutputVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;

using drake::multibody::JacobianWrtVariable;
using drake::trajectories::PiecewisePolynomial;

using drake::MatrixX;

namespace dairlib {
namespace cassie {
namespace osc {

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double vel_scale_rot,
    double vel_scale_trans_sagital, double vel_scale_trans_lateral)
    : HighLevelCommand(plant, context) {
  cassie_out_port_ =
      this->DeclareAbstractInputPort("lcmt_cassie_output",
                                     drake::Value<dairlib::lcmt_cassie_out>{})
          .get_index();
  //  use_radio_command_ = true;
  high_level_mode_ = radio;

  vel_scale_rot_ = vel_scale_rot;
  vel_scale_trans_sagital_ = vel_scale_trans_sagital;
  vel_scale_trans_lateral_ = vel_scale_trans_lateral;
}

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double kp_yaw, double kd_yaw,
    double vel_max_yaw, double kp_pos_sagital, double kd_pos_sagital,
    double vel_max_sagital, double kp_pos_lateral, double kd_pos_lateral,
    double vel_max_lateral, double target_pos_offset,
    const Vector2d& global_target_position,
    const Vector2d& params_of_no_turning)
    : HighLevelCommand(plant, context) {
  //  use_radio_command_ = false;
  high_level_mode_ = desired_xy_position;

  kp_yaw_ = kp_yaw;
  kd_yaw_ = kd_yaw;
  vel_max_yaw_ = vel_max_yaw;
  kp_pos_sagital_ = kp_pos_sagital;
  kd_pos_sagital_ = kd_pos_sagital;
  vel_max_sagital_ = vel_max_sagital;
  target_pos_offset_ = target_pos_offset;
  kp_pos_lateral_ = kp_pos_lateral;
  kd_pos_lateral_ = kd_pos_lateral;
  vel_max_lateral_ = vel_max_lateral;

  global_target_position_ = global_target_position;
  params_of_no_turning_ = params_of_no_turning;
}

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")) {
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();

  yaw_port_ =
      this->DeclareVectorOutputPort("pelvis_yaw", BasicVector<double>(1),
                                    &HighLevelCommand::CopyHeadingAngle)
          .get_index();
  xy_port_ =
      this->DeclareVectorOutputPort("pelvis_xy", BasicVector<double>(2),
                                    &HighLevelCommand::CopyDesiredHorizontalVel)
          .get_index();
  // Declare update event
  DeclarePerStepDiscreteUpdateEvent(&HighLevelCommand::DiscreteVariableUpdate);

  // Discrete state which stores the desired yaw velocity
  des_vel_idx_ = DeclareDiscreteState(VectorXd::Zero(3));
}

void HighLevelCommand::SetOpenLoopVelCommandTraj() {
  high_level_mode_ = open_loop_vel_command_traj;

  std::vector<double> breaks = {0};
  std::vector<MatrixXd> knots = {(MatrixX<double>(3, 1) << 0, 0, 0).finished()};
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0, 0, 0).finished());
  breaks.push_back(breaks.back() + 5);
  knots.push_back((MatrixX<double>(3, 1) << 0, 2, 0).finished());
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0.7, 2, 0).finished());
  breaks.push_back(breaks.back() + 1.5);
  knots.push_back((MatrixX<double>(3, 1) << 0.7, 2, 0).finished());
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0, 2, 0).finished());
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0, 2, 0).finished());
  breaks.push_back(breaks.back() + 1);
  knots.push_back((MatrixX<double>(3, 1) << 0, 0, 0).finished());
  breaks.push_back(breaks.back() + 10000000);
  knots.push_back((MatrixX<double>(3, 1) << 0, 0, 0).finished());

  // Construct the PiecewisePolynomial.
  desired_vel_command_traj_ =
      PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
};

void HighLevelCommand::SetDesiredXYTraj(
    const multibody::ViewFrame<double>* view_frame) {
  high_level_mode_ = desired_xy_traj;
  view_frame_ = view_frame;

  // Set output port for Z traj
  slope_port_ =
      this->DeclareVectorOutputPort("slope", BasicVector<double>(1),
                                    &HighLevelCommand::CopyFeedforwardSlope)
          .get_index();

  // Set x y set points
  /*std::vector<double> breaks = {0};
  std::vector<std::vector<double>> knots_vec = {{0, 0}};
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({0, 0});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({2, 0});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({4, 2});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({4, 4});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({2, 6});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({0, 6});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({0, 6});
  breaks.back() = 10000000;*/

  // Traj:  2023-05-28 10h52m14s: Straight 2.5 meters -> Straight 2.5 meters #2
  // -> Turn 180 degrees -> Straight 2.5 meters #3 -> Straight 2.5 meters #4
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=2.500, dx/dt=1.0
  // 6.500: {3.500, 0.000};   dt=2.500, dx/dt=1.0
  // 9.000: {6.000, 0.000};   dt=0.166, dx/dt=1.0
  // 9.166: {6.166, 0.000};   dt=0.331, dx/dt=1.0
  // 9.497: {6.493, 0.055};   dt=0.331, dx/dt=1.0
  // 9.829: {6.806, 0.162};   dt=0.331, dx/dt=1.0
  // 10.160: {7.098, 0.320};   dt=0.331, dx/dt=1.0
  // 10.492: {7.359, 0.524};   dt=0.331, dx/dt=1.0
  // 10.823: {7.584, 0.767};   dt=0.331, dx/dt=1.0
  // 11.154: {7.765, 1.045};   dt=0.331, dx/dt=1.0
  // 11.486: {7.898, 1.348};   dt=0.331, dx/dt=1.0
  // 11.817: {7.979, 1.670};   dt=0.331, dx/dt=1.0
  // 12.149: {8.007, 2.000};   dt=0.331, dx/dt=1.0
  // 12.480: {7.979, 2.330};   dt=0.331, dx/dt=1.0
  // 12.812: {7.898, 2.652};   dt=0.331, dx/dt=1.0
  // 13.143: {7.765, 2.955};   dt=0.331, dx/dt=1.0
  // 13.475: {7.584, 3.233};   dt=0.331, dx/dt=1.0
  // 13.806: {7.359, 3.476};   dt=0.331, dx/dt=1.0
  // 14.137: {7.098, 3.680};   dt=0.331, dx/dt=1.0
  // 14.469: {6.806, 3.838};   dt=0.331, dx/dt=1.0
  // 14.800: {6.493, 3.945};   dt=0.331, dx/dt=1.0
  // 15.132: {6.166, 4.000};   dt=0.166, dx/dt=1.0
  // 15.298: {6.000, 4.000};   dt=2.500, dx/dt=1.0
  // 17.798: {3.500, 4.000};   dt=2.500, dx/dt=1.0
  // 20.298: {1.000, 4.000};   dt=2.000, dx/dt=0.5
  // 20.298: {0.000, 4.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000,  6.500,  9.000,  9.166,
                                9.497,  9.829,  10.160, 10.492, 10.823, 11.154,
                                11.486, 11.817, 12.149, 12.480, 12.812, 13.143,
                                13.475, 13.806, 14.137, 14.469, 14.800, 15.132,
                                15.298, 17.798, 20.298, 22.298};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {1.000, 0.000}, {3.500, 0.000},
      {6.000, 0.000}, {6.166, 0.000}, {6.493, 0.055}, {6.806, 0.162},
      {7.098, 0.320}, {7.359, 0.524}, {7.584, 0.767}, {7.765, 1.045},
      {7.898, 1.348}, {7.979, 1.670}, {8.007, 2.000}, {7.979, 2.330},
      {7.898, 2.652}, {7.765, 2.955}, {7.584, 3.233}, {7.359, 3.476},
      {7.098, 3.680}, {6.806, 3.838}, {6.493, 3.945}, {6.166, 4.000},
      {6.000, 4.000}, {3.500, 4.000}, {1.000, 4.000}, {0.000, 4.000}};*/

  // Traj:  2023-05-28 11h31m35s: Straight 2.5 meters -> Straight 2.5 meters #2
  // -> Turn 180 degrees -> Straight 2.5 meters #3 -> Straight 2.5 meters #4
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=1.250, dx/dt=2.0
  // 5.250: {3.500, 0.000};   dt=1.250, dx/dt=2.0
  // 6.500: {6.000, 0.000};   dt=0.083, dx/dt=2.0
  // 6.583: {6.166, 0.000};   dt=0.166, dx/dt=2.0
  // 6.749: {6.493, 0.055};   dt=0.166, dx/dt=2.0
  // 6.914: {6.806, 0.162};   dt=0.166, dx/dt=2.0
  // 7.080: {7.098, 0.320};   dt=0.166, dx/dt=2.0
  // 7.246: {7.359, 0.524};   dt=0.166, dx/dt=2.0
  // 7.411: {7.584, 0.767};   dt=0.166, dx/dt=2.0
  // 7.577: {7.765, 1.045};   dt=0.166, dx/dt=2.0
  // 7.743: {7.898, 1.348};   dt=0.166, dx/dt=2.0
  // 7.909: {7.979, 1.670};   dt=0.166, dx/dt=2.0
  // 8.074: {8.007, 2.000};   dt=0.166, dx/dt=2.0
  // 8.240: {7.979, 2.330};   dt=0.166, dx/dt=2.0
  // 8.406: {7.898, 2.652};   dt=0.166, dx/dt=2.0
  // 8.572: {7.765, 2.955};   dt=0.166, dx/dt=2.0
  // 8.737: {7.584, 3.233};   dt=0.166, dx/dt=2.0
  // 8.903: {7.359, 3.476};   dt=0.166, dx/dt=2.0
  // 9.069: {7.098, 3.680};   dt=0.166, dx/dt=2.0
  // 9.234: {6.806, 3.838};   dt=0.166, dx/dt=2.0
  // 9.400: {6.493, 3.945};   dt=0.166, dx/dt=2.0
  // 9.566: {6.166, 4.000};   dt=0.083, dx/dt=2.0
  // 9.649: {6.000, 4.000};   dt=1.250, dx/dt=2.0
  // 10.899: {3.500, 4.000};   dt=1.250, dx/dt=2.0
  // 12.149: {1.000, 4.000};   dt=2.000, dx/dt=0.5
  // 12.149: {0.000, 4.000}
  /*std::vector<double> breaks = {
      0.000, 2.000, 4.000, 5.250, 6.500, 6.583,  6.749,  6.914, 7.080, 7.246,
      7.411, 7.577, 7.743, 7.909, 8.074, 8.240,  8.406,  8.572, 8.737, 8.903,
      9.069, 9.234, 9.400, 9.566, 9.649, 10.899, 12.149, 14.149};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {1.000, 0.000}, {3.500, 0.000},
      {6.000, 0.000}, {6.166, 0.000}, {6.493, 0.055}, {6.806, 0.162},
      {7.098, 0.320}, {7.359, 0.524}, {7.584, 0.767}, {7.765, 1.045},
      {7.898, 1.348}, {7.979, 1.670}, {8.007, 2.000}, {7.979, 2.330},
      {7.898, 2.652}, {7.765, 2.955}, {7.584, 3.233}, {7.359, 3.476},
      {7.098, 3.680}, {6.806, 3.838}, {6.493, 3.945}, {6.166, 4.000},
      {6.000, 4.000}, {3.500, 4.000}, {1.000, 4.000}, {0.000, 4.000}};*/

  // Traj:  2023-05-28 13h01m16s: Straight 5.0 meters -> Turn -180 degrees (2.0
  // m radius)  -> Straight 5.0 meters #2
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000};   dt=0.083, dx/dt=2.0
  // 6.583: {6.166, 0.000};   dt=0.166, dx/dt=2.0
  // 6.749: {6.493, -0.055};   dt=0.166, dx/dt=2.0
  // 6.914: {6.806, -0.162};   dt=0.166, dx/dt=2.0
  // 7.080: {7.098, -0.320};   dt=0.166, dx/dt=2.0
  // 7.246: {7.359, -0.524};   dt=0.166, dx/dt=2.0
  // 7.411: {7.584, -0.767};   dt=0.166, dx/dt=2.0
  // 7.577: {7.765, -1.045};   dt=0.166, dx/dt=2.0
  // 7.743: {7.898, -1.348};   dt=0.166, dx/dt=2.0
  // 7.909: {7.979, -1.670};   dt=0.166, dx/dt=2.0
  // 8.074: {8.007, -2.000};   dt=0.166, dx/dt=2.0
  // 8.240: {7.979, -2.330};   dt=0.166, dx/dt=2.0
  // 8.406: {7.898, -2.652};   dt=0.166, dx/dt=2.0
  // 8.572: {7.765, -2.955};   dt=0.166, dx/dt=2.0
  // 8.737: {7.584, -3.233};   dt=0.166, dx/dt=2.0
  // 8.903: {7.359, -3.476};   dt=0.166, dx/dt=2.0
  // 9.069: {7.098, -3.680};   dt=0.166, dx/dt=2.0
  // 9.234: {6.806, -3.838};   dt=0.166, dx/dt=2.0
  // 9.400: {6.493, -3.945};   dt=0.166, dx/dt=2.0
  // 9.566: {6.166, -4.000};   dt=0.083, dx/dt=2.0
  // 9.649: {6.000, -4.000};   dt=2.500, dx/dt=2.0
  // 12.149: {1.000, -4.000};   dt=2.000, dx/dt=0.5
  // 12.149: {0.000, -4.000}
  /*std::vector<double> breaks = {
      0.000, 2.000, 4.000, 6.500, 6.583, 6.749, 6.914,  7.080, 7.246,
      7.411, 7.577, 7.743, 7.909, 8.074, 8.240, 8.406,  8.572, 8.737,
      8.903, 9.069, 9.234, 9.400, 9.566, 9.649, 12.149, 14.149};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000},  {0.000, 0.000},  {1.000, 0.000},  {6.000, 0.000},
      {6.166, 0.000},  {6.493, -0.055}, {6.806, -0.162}, {7.098, -0.320},
      {7.359, -0.524}, {7.584, -0.767}, {7.765, -1.045}, {7.898, -1.348},
      {7.979, -1.670}, {8.007, -2.000}, {7.979, -2.330}, {7.898, -2.652},
      {7.765, -2.955}, {7.584, -3.233}, {7.359, -3.476}, {7.098, -3.680},
      {6.806, -3.838}, {6.493, -3.945}, {6.166, -4.000}, {6.000, -4.000},
      {1.000, -4.000}, {0.000, -4.000}};*/

  //////////////////////////////////////////////////////////////////////////////
  // Traj:  2023-05-28 12h46m42s: Straight 5.0 meters -> Turn 180 degrees (1.0 m
  // radius)  -> Straight 5.0 meters #2 0.000: {0.000, 0.000};   dt=2.000,
  // dx/dt=0.0 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5 4.000: {1.000,
  // 0.000};   dt=2.500, dx/dt=2.0 6.500: {6.000, 0.000};   dt=0.041, dx/dt=2.0
  // 6.541: {6.083, 0.000};   dt=0.083, dx/dt=2.0
  // 6.624: {6.246, 0.027};   dt=0.083, dx/dt=2.0
  // 6.707: {6.403, 0.081};   dt=0.083, dx/dt=2.0
  // 6.790: {6.549, 0.160};   dt=0.083, dx/dt=2.0
  // 6.873: {6.680, 0.262};   dt=0.083, dx/dt=2.0
  // 6.956: {6.792, 0.384};   dt=0.083, dx/dt=2.0
  // 7.039: {6.882, 0.522};   dt=0.083, dx/dt=2.0
  // 7.121: {6.949, 0.674};   dt=0.083, dx/dt=2.0
  // 7.204: {6.990, 0.835};   dt=0.083, dx/dt=2.0
  // 7.287: {7.003, 1.000};   dt=0.083, dx/dt=2.0
  // 7.370: {6.990, 1.165};   dt=0.083, dx/dt=2.0
  // 7.453: {6.949, 1.326};   dt=0.083, dx/dt=2.0
  // 7.536: {6.882, 1.478};   dt=0.083, dx/dt=2.0
  // 7.619: {6.792, 1.616};   dt=0.083, dx/dt=2.0
  // 7.702: {6.680, 1.738};   dt=0.083, dx/dt=2.0
  // 7.784: {6.549, 1.840};   dt=0.083, dx/dt=2.0
  // 7.867: {6.403, 1.919};   dt=0.083, dx/dt=2.0
  // 7.950: {6.246, 1.973};   dt=0.083, dx/dt=2.0
  // 8.033: {6.083, 2.000};   dt=0.041, dx/dt=2.0
  // 8.074: {6.000, 2.000};   dt=2.500, dx/dt=2.0
  // 10.574: {1.000, 2.000};   dt=2.000, dx/dt=0.5
  // 10.574: {0.000, 2.000}
  /*std::vector<double> breaks = {
      0.000, 2.000, 4.000, 6.500, 6.541, 6.624, 6.707,  6.790, 6.873,
      6.956, 7.039, 7.121, 7.204, 7.287, 7.370, 7.453,  7.536, 7.619,
      7.702, 7.784, 7.867, 7.950, 8.033, 8.074, 10.574, 12.574};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {1.000, 0.000}, {6.000, 0.000},
      {6.083, 0.000}, {6.246, 0.027}, {6.403, 0.081}, {6.549, 0.160},
      {6.680, 0.262}, {6.792, 0.384}, {6.882, 0.522}, {6.949, 0.674},
      {6.990, 0.835}, {7.003, 1.000}, {6.990, 1.165}, {6.949, 1.326},
      {6.882, 1.478}, {6.792, 1.616}, {6.680, 1.738}, {6.549, 1.840},
      {6.403, 1.919}, {6.246, 1.973}, {6.083, 2.000}, {6.000, 2.000},
      {1.000, 2.000}, {0.000, 2.000}};*/

  // Traj:  2023-05-28 20h47m49s: Straight 5.0 meters -> Turn 90 degrees (1.0 m
  // radius)  -> Straight 5.0 meters #2
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000};   dt=0.044, dx/dt=2.0
  // 6.544: {6.087, 0.000};   dt=0.087, dx/dt=2.0
  // 6.631: {6.260, 0.030};   dt=0.087, dx/dt=2.0
  // 6.719: {6.424, 0.090};   dt=0.087, dx/dt=2.0
  // 6.806: {6.576, 0.178};   dt=0.087, dx/dt=2.0
  // 6.894: {6.710, 0.290};   dt=0.087, dx/dt=2.0
  // 6.981: {6.822, 0.424};   dt=0.087, dx/dt=2.0
  // 7.069: {6.910, 0.576};   dt=0.087, dx/dt=2.0
  // 7.156: {6.970, 0.740};   dt=0.087, dx/dt=2.0
  // 7.244: {7.000, 0.913};   dt=0.044, dx/dt=2.0
  // 7.287: {7.000, 1.000};   dt=2.500, dx/dt=2.0
  // 9.787: {7.000, 6.000};   dt=2.000, dx/dt=0.5
  // 9.787: {7.000, 7.000}
  /*std::vector<double> breaks = {0.000, 2.000, 4.000, 6.500, 6.544, 6.631,
                                6.719, 6.806, 6.894, 6.981, 7.069, 7.156,
                                7.244, 7.287, 9.787, 11.787};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {1.000, 0.000}, {6.000, 0.000},
      {6.087, 0.000}, {6.260, 0.030}, {6.424, 0.090}, {6.576, 0.178},
      {6.710, 0.290}, {6.822, 0.424}, {6.910, 0.576}, {6.970, 0.740},
      {7.000, 0.913}, {7.000, 1.000}, {7.000, 6.000}, {7.000, 7.000}};*/

  //////////////////////////////////////////////////////////////////////////////
  // Traj:  2023-05-28 20h21m30s: Straight 5.0 meters -> Turn 180 degrees (2.0 m
  // radius)  -> Turn -180 degrees (2.0 m radius)  -> Turn 180 degrees (2.0 m
  // radius)  #2 -> Straight 5.0 meters #2
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000};   dt=0.083, dx/dt=2.0
  // 6.583: {6.166, 0.000};   dt=0.166, dx/dt=2.0
  // 6.749: {6.493, 0.055};   dt=0.166, dx/dt=2.0
  // 6.914: {6.806, 0.162};   dt=0.166, dx/dt=2.0
  // 7.080: {7.098, 0.320};   dt=0.166, dx/dt=2.0
  // 7.246: {7.359, 0.524};   dt=0.166, dx/dt=2.0
  // 7.411: {7.584, 0.767};   dt=0.166, dx/dt=2.0
  // 7.577: {7.765, 1.045};   dt=0.166, dx/dt=2.0
  // 7.743: {7.898, 1.348};   dt=0.166, dx/dt=2.0
  // 7.909: {7.979, 1.670};   dt=0.166, dx/dt=2.0
  // 8.074: {8.007, 2.000};   dt=0.166, dx/dt=2.0
  // 8.240: {7.979, 2.330};   dt=0.166, dx/dt=2.0
  // 8.406: {7.898, 2.652};   dt=0.166, dx/dt=2.0
  // 8.572: {7.765, 2.955};   dt=0.166, dx/dt=2.0
  // 8.737: {7.584, 3.233};   dt=0.166, dx/dt=2.0
  // 8.903: {7.359, 3.476};   dt=0.166, dx/dt=2.0
  // 9.069: {7.098, 3.680};   dt=0.166, dx/dt=2.0
  // 9.234: {6.806, 3.838};   dt=0.166, dx/dt=2.0
  // 9.400: {6.493, 3.945};   dt=0.166, dx/dt=2.0
  // 9.566: {6.166, 4.000};   dt=0.083, dx/dt=2.0
  // 9.649: {6.000, 4.000};   dt=0.083, dx/dt=2.0
  // 9.732: {5.834, 4.000};   dt=0.166, dx/dt=2.0
  // 9.897: {5.507, 4.055};   dt=0.166, dx/dt=2.0
  // 10.063: {5.194, 4.162};   dt=0.166, dx/dt=2.0
  // 10.229: {4.902, 4.320};   dt=0.166, dx/dt=2.0
  // 10.395: {4.641, 4.524};   dt=0.166, dx/dt=2.0
  // 10.560: {4.416, 4.767};   dt=0.166, dx/dt=2.0
  // 10.726: {4.235, 5.045};   dt=0.166, dx/dt=2.0
  // 10.892: {4.102, 5.348};   dt=0.166, dx/dt=2.0
  // 11.057: {4.021, 5.670};   dt=0.166, dx/dt=2.0
  // 11.223: {3.993, 6.000};   dt=0.166, dx/dt=2.0
  // 11.389: {4.021, 6.330};   dt=0.166, dx/dt=2.0
  // 11.555: {4.102, 6.652};   dt=0.166, dx/dt=2.0
  // 11.720: {4.235, 6.955};   dt=0.166, dx/dt=2.0
  // 11.886: {4.416, 7.233};   dt=0.166, dx/dt=2.0
  // 12.052: {4.641, 7.476};   dt=0.166, dx/dt=2.0
  // 12.218: {4.902, 7.680};   dt=0.166, dx/dt=2.0
  // 12.383: {5.194, 7.838};   dt=0.166, dx/dt=2.0
  // 12.549: {5.507, 7.945};   dt=0.166, dx/dt=2.0
  // 12.715: {5.834, 8.000};   dt=0.083, dx/dt=2.0
  // 12.798: {6.000, 8.000};   dt=0.083, dx/dt=2.0
  // 12.880: {6.166, 8.000};   dt=0.166, dx/dt=2.0
  // 13.046: {6.493, 8.055};   dt=0.166, dx/dt=2.0
  // 13.212: {6.806, 8.162};   dt=0.166, dx/dt=2.0
  // 13.378: {7.098, 8.320};   dt=0.166, dx/dt=2.0
  // 13.543: {7.359, 8.524};   dt=0.166, dx/dt=2.0
  // 13.709: {7.584, 8.767};   dt=0.166, dx/dt=2.0
  // 13.875: {7.765, 9.045};   dt=0.166, dx/dt=2.0
  // 14.040: {7.898, 9.348};   dt=0.166, dx/dt=2.0
  // 14.206: {7.979, 9.670};   dt=0.166, dx/dt=2.0
  // 14.372: {8.007, 10.000};   dt=0.166, dx/dt=2.0
  // 14.538: {7.979, 10.330};   dt=0.166, dx/dt=2.0
  // 14.703: {7.898, 10.652};   dt=0.166, dx/dt=2.0
  // 14.869: {7.765, 10.955};   dt=0.166, dx/dt=2.0
  // 15.035: {7.584, 11.233};   dt=0.166, dx/dt=2.0
  // 15.201: {7.359, 11.476};   dt=0.166, dx/dt=2.0
  // 15.366: {7.098, 11.680};   dt=0.166, dx/dt=2.0
  // 15.532: {6.806, 11.838};   dt=0.166, dx/dt=2.0
  // 15.698: {6.493, 11.945};   dt=0.166, dx/dt=2.0
  // 15.863: {6.166, 12.000};   dt=0.083, dx/dt=2.0
  // 15.946: {6.000, 12.000};   dt=2.500, dx/dt=2.0
  // 18.446: {1.000, 12.000};   dt=2.000, dx/dt=0.5
  // 18.446: {0.000, 12.000}
  /*std::vector<double> breaks = {
      0.000,  2.000,  4.000,  6.500,  6.583,  6.749,  6.914,  7.080,  7.246,
      7.411,  7.577,  7.743,  7.909,  8.074,  8.240,  8.406,  8.572,  8.737,
      8.903,  9.069,  9.234,  9.400,  9.566,  9.649,  9.732,  9.897,  10.063,
      10.229, 10.395, 10.560, 10.726, 10.892, 11.057, 11.223, 11.389, 11.555,
      11.720, 11.886, 12.052, 12.218, 12.383, 12.549, 12.715, 12.798, 12.880,
      13.046, 13.212, 13.378, 13.543, 13.709, 13.875, 14.040, 14.206, 14.372,
      14.538, 14.703, 14.869, 15.035, 15.201, 15.366, 15.532, 15.698, 15.863,
      15.946, 18.446, 20.446};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000},  {0.000, 0.000},  {1.000, 0.000},  {6.000, 0.000},
      {6.166, 0.000},  {6.493, 0.055},  {6.806, 0.162},  {7.098, 0.320},
      {7.359, 0.524},  {7.584, 0.767},  {7.765, 1.045},  {7.898, 1.348},
      {7.979, 1.670},  {8.007, 2.000},  {7.979, 2.330},  {7.898, 2.652},
      {7.765, 2.955},  {7.584, 3.233},  {7.359, 3.476},  {7.098, 3.680},
      {6.806, 3.838},  {6.493, 3.945},  {6.166, 4.000},  {6.000, 4.000},
      {5.834, 4.000},  {5.507, 4.055},  {5.194, 4.162},  {4.902, 4.320},
      {4.641, 4.524},  {4.416, 4.767},  {4.235, 5.045},  {4.102, 5.348},
      {4.021, 5.670},  {3.993, 6.000},  {4.021, 6.330},  {4.102, 6.652},
      {4.235, 6.955},  {4.416, 7.233},  {4.641, 7.476},  {4.902, 7.680},
      {5.194, 7.838},  {5.507, 7.945},  {5.834, 8.000},  {6.000, 8.000},
      {6.166, 8.000},  {6.493, 8.055},  {6.806, 8.162},  {7.098, 8.320},
      {7.359, 8.524},  {7.584, 8.767},  {7.765, 9.045},  {7.898, 9.348},
      {7.979, 9.670},  {8.007, 10.000}, {7.979, 10.330}, {7.898, 10.652},
      {7.765, 10.955}, {7.584, 11.233}, {7.359, 11.476}, {7.098, 11.680},
      {6.806, 11.838}, {6.493, 11.945}, {6.166, 12.000}, {6.000, 12.000},
      {1.000, 12.000}, {0.000, 12.000}};*/

  //////////////////////////////////////////////////////////////////////////////
  // Traj:  2023-05-28 22h07m31s: Straight 5.0 meters -> Ramp 10% slope and 5.0
  // m -> Straight 5.0 meters #2
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000};   dt=2.512, dx/dt=2.0
  // 9.012: {11.000, 0.000};   dt=2.500, dx/dt=2.0
  // 11.512: {16.000, 0.000};   dt=2.000, dx/dt=0.5
  // 11.512: {17.000, 0.000}
  /*std::vector<double> breaks = {0.000, 2.000,  4.000, 6.500,
                                9.012, 11.512, 13.512};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000},  {0.000, 0.000},  {1.000, 0.000}, {6.000, 0.000},
      {11.000, 0.000}, {16.000, 0.000}, {17.000, 0.000}};*/

  // Traj:  2023-05-28 22h28m27s: Straight 5.0 meters -> Turn 90 degrees (2.0 m
  // radius)  -> Ramp 10% slope and 2.5 m -> Straight 5.0 meters #2
  // Turn then ramp
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000};   dt=0.087, dx/dt=2.0
  // 6.587: {6.175, 0.000};   dt=0.175, dx/dt=2.0
  // 6.762: {6.520, 0.061};   dt=0.175, dx/dt=2.0
  // 6.937: {6.848, 0.180};   dt=0.175, dx/dt=2.0
  // 7.112: {7.152, 0.355};   dt=0.175, dx/dt=2.0
  // 7.287: {7.420, 0.580};   dt=0.175, dx/dt=2.0
  // 7.462: {7.645, 0.848};   dt=0.175, dx/dt=2.0
  // 7.637: {7.820, 1.152};   dt=0.175, dx/dt=2.0
  // 7.812: {7.939, 1.480};   dt=0.175, dx/dt=2.0
  // 7.987: {8.000, 1.825};   dt=0.087, dx/dt=2.0
  // 8.075: {8.000, 2.000};   dt=1.256, dx/dt=2.0
  // 9.331: {8.000, 4.500};   dt=2.500, dx/dt=2.0
  // 11.831: {8.000, 9.500};   dt=2.000, dx/dt=0.5
  // 11.831: {8.000, 10.500}
  /*std::vector<double> breaks = {0.000, 2.000, 4.000, 6.500,  6.587, 6.762,
                                6.937, 7.112, 7.287, 7.462,  7.637, 7.812,
                                7.987, 8.075, 9.331, 11.831, 13.831};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {1.000, 0.000}, {6.000, 0.000},
      {6.175, 0.000}, {6.520, 0.061}, {6.848, 0.180}, {7.152, 0.355},
      {7.420, 0.580}, {7.645, 0.848}, {7.820, 1.152}, {7.939, 1.480},
      {8.000, 1.825}, {8.000, 2.000}, {8.000, 4.500}, {8.000, 9.500},
      {8.000, 10.500}};*/

  //////////////////////////////////////////////////////////////////////////////
  // Traj:  2023-05-29 14h45m38s: Straight 5.0 meters -> Ramp 20.0% slope and 10
  // m -> Straight 5.0 meters #2
  // Straight line 20% ramp; high speed
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000, 0.000};   dt=5.099, dx/dt=2.0
  // 11.599: {16.000, 0.000, 2.000};   dt=2.500, dx/dt=2.0
  // 14.099: {21.000, 0.000, 2.000};   dt=2.000, dx/dt=0.5
  // 14.099: {22.000, 0.000, 2.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000, 6.500,
                                11.599, 14.099, 16.099};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000}, {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000}, {16.000, 0.000, 2.000}, {21.000, 0.000, 2.000},
      {22.000, 0.000, 2.000}};*/

  // Traj:  2023-05-28 22h58m51s: Straight 5.0 meters -> Ramp 20.0% slope and 10
  // m -> Straight 5.0 meters #2
  // Straight line 20% ramp; slower speed
  // 0.000: {0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000};   dt=10.000, dx/dt=0.5
  // 14.000: {6.000, 0.000};   dt=20.396, dx/dt=0.5
  // 34.396: {16.000, 0.000};   dt=10.000, dx/dt=0.5
  // 44.396: {21.000, 0.000};   dt=2.000, dx/dt=0.5
  // 44.396: {22.000, 0.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000, 14.000,
                                34.396, 44.396, 46.396};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000},  {0.000, 0.000},  {1.000, 0.000}, {6.000, 0.000},
      {16.000, 0.000}, {21.000, 0.000}, {22.000, 0.000}};*/

  // Traj:  2023-05-29 00h04m31s: Straight 5.0 meters -> Ramp 20.0% slope and 10
  // m -> Straight 5.0 meters #2
  // Straight line 20% ramp; slower speed; 3D waypoint specification
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=10.000, dx/dt=0.5
  // 14.000: {6.000, 0.000, 0.000};   dt=20.396, dx/dt=0.5
  // 34.396: {16.000, 0.000, 2.000};   dt=10.000, dx/dt=0.5
  // 44.396: {21.000, 0.000, 2.000};   dt=2.000, dx/dt=0.5
  // 44.396: {22.000, 0.000, 2.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000, 14.000,
                                34.396, 44.396, 46.396};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000}, {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000}, {16.000, 0.000, 2.000}, {21.000, 0.000, 2.000},
      {22.000, 0.000, 2.000}};*/

  //////////////////////////////////////////////////////////////////////////////
  // Traj:  2023-05-29 15h24m37s: Straight 5.0 meters -> Ramp 35.0% slope and 10
  // m -> Straight 5.0 meters #2
  //  Straight line 35% ramp; higher speed
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=5.000, dx/dt=1.0
  // 9.000: {6.000, 0.000, 0.000};   dt=10.595, dx/dt=1.0
  // 19.595: {16.000, 0.000, 3.500};   dt=5.000, dx/dt=1.0
  // 24.595: {21.000, 0.000, 3.500};   dt=2.000, dx/dt=0.5
  // 24.595: {22.000, 0.000, 3.500}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000, 9.000,
                                19.595, 24.595, 26.595};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000}, {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000}, {16.000, 0.000, 3.500}, {21.000, 0.000, 3.500},
      {22.000, 0.000, 3.500}};
  cout << "Remember to increase the mid foot height to 0.15m\n";*/

  // Traj:  2023-05-29 16h04m07s: Straight 5.0 meters -> Ramp 50% slope
  // (base 10.0m) -> Straight 5.0 meters #2
  //  Straight line 50% ramp
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=5.000, dx/dt=1.0
  // 9.000: {6.000, 0.000, 0.000};   dt=11.180, dx/dt=1.0
  // 20.180: {16.000, 0.000, 5.000};   dt=5.000, dx/dt=1.0
  // 25.180: {21.000, 0.000, 5.000};   dt=2.500, dx/dt=0.5
  // 25.180: {22.250, 0.000, 5.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000, 9.000,
                                20.180, 25.180, 27.680};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000}, {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000}, {16.000, 0.000, 5.000}, {21.000, 0.000, 5.000},
      {22.250, 0.000, 5.000}};
  cout << "wasn't able to do 50% ramp; even with sim friction coefficent 1.5, "
          "and mid foot height 0.25m\n";*/

  // Traj:  2023-05-29 16h16m15s: Straight 5.0 meters -> Ramp 50% slope
  // (base 10.0m) -> Straight 5.0 meters #2
  //  Straight line 50% ramp; slow speed
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=10.000, dx/dt=0.5
  // 14.000: {6.000, 0.000, 0.000};   dt=22.361, dx/dt=0.5
  // 36.361: {16.000, 0.000, 5.000};   dt=10.000, dx/dt=0.5
  // 46.361: {21.000, 0.000, 5.000};   dt=2.500, dx/dt=0.5
  // 46.361: {22.250, 0.000, 5.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000, 14.000,
                                36.361, 46.361, 48.861};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000}, {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000}, {16.000, 0.000, 5.000}, {21.000, 0.000, 5.000},
      {22.250, 0.000, 5.000}};
  cout << "optimal ROM walks very well with mid foot height 0.25m\n";*/

  //////////////////////////////////////////////////////////////////////////////
  // Traj: 2023-05-29 17h10m02s: Straight 2.5 meters -> Ramp -20% slope
  // (base 2.5m) -> Ramp -20% slope (base 10.0m) -> Ramp -20% slope (base 2.5m)
  // #2 -> Straight 2.5 meters #2
  // 20% ramp down
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=5.000, dx/dt=0.5
  // 9.000: {3.500, 0.000, 0.000};   dt=5.099, dx/dt=0.5
  // 14.099: {6.000, 0.000, -0.500};   dt=5.099, dx/dt=2.0
  // 19.198: {16.000, 0.000, -2.500};   dt=5.099, dx/dt=0.5
  // 24.297: {18.500, 0.000, -3.000};   dt=5.000, dx/dt=0.5
  // 29.297: {21.000, 0.000, -3.000};   dt=2.500, dx/dt=0.5
  // 29.297: {22.250, 0.000, -3.000}
  std::vector<double> breaks = {0.000,  2.000,  4.000,  9.000, 14.099,
                                19.198, 24.297, 29.297, 31.797};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000},   {0.000, 0.000, 0.000},
      {1.000, 0.000, 0.000},   {3.500, 0.000, 0.000},
      {6.000, 0.000, -0.500},  {16.000, 0.000, -2.500},
      {18.500, 0.000, -3.000}, {21.000, 0.000, -3.000},
      {22.250, 0.000, -3.000}};

  //////////////////////////////////////////////////////////////////////////////
  // Traj:  2023-05-29 15h00m57s: Straight 5.0 meters -> Ramp 10.0% slope
  // and 10 m -> Ramp 20.0% slope and 10 m -> Straight 5.0 meters #2
  //  Two ramps with progressive incline; higher speed
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=5.000, dx/dt=1.0
  // 9.000: {6.000, 0.000, 0.000};   dt=10.050, dx/dt=1.0
  // 19.050: {16.000, 0.000, 1.000};   dt=10.198, dx/dt=1.0
  // 29.248: {26.000, 0.000, 3.000};   dt=5.000, dx/dt=1.0
  // 34.248: {31.000, 0.000, 3.000};   dt=2.000, dx/dt=0.5
  // 34.248: {32.000, 0.000, 3.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000,  9.000,
                                19.050, 29.248, 34.248, 36.248};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000},  {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000},  {16.000, 0.000, 1.000}, {26.000,
  0.000, 3.000}, {31.000, 0.000, 3.000}, {32.000, 0.000, 3.000}};*/

  // Traj:  2023-05-29 15h15m28s: Straight 5.0 meters -> Ramp 10.0% slope
  // and 10 m -> Ramp 20.0% slope and 10 m -> Straight 5.0 meters #2
  //  Two ramps with progressive incline; high speed
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000, 0.000};   dt=5.025, dx/dt=2.0
  // 11.525: {16.000, 0.000, 1.000};   dt=5.099, dx/dt=2.0
  // 16.624: {26.000, 0.000, 3.000};   dt=2.500, dx/dt=2.0
  // 19.124: {31.000, 0.000, 3.000};   dt=2.000, dx/dt=0.5
  // 19.124: {32.000, 0.000, 3.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000,  6.500,
                                11.525, 16.624, 19.124, 21.124};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000},  {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000},  {16.000, 0.000, 1.000}, {26.000,
  0.000, 3.000}, {31.000, 0.000, 3.000}, {32.000, 0.000, 3.000}};*/

  // Traj:  2023-05-29 15h43m14s: Straight 5.0 meters -> Ramp 10.0% slope
  // and 5 m -> Ramp 20.0% slope and 10 m -> Ramp 10.0% slope and 5 m #2 ->
  // Straight 5.0 meters #2
  //  10-20-10% ramps; high speed
  // 0.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.0
  // 2.000: {0.000, 0.000, 0.000};   dt=2.000, dx/dt=0.5
  // 4.000: {1.000, 0.000, 0.000};   dt=2.500, dx/dt=2.0
  // 6.500: {6.000, 0.000, 0.000};   dt=2.512, dx/dt=2.0
  // 9.012: {11.000, 0.000, 0.500};   dt=5.099, dx/dt=2.0
  // 14.111: {21.000, 0.000, 2.500};   dt=2.512, dx/dt=2.0
  // 16.624: {26.000, 0.000, 3.000};   dt=2.500, dx/dt=2.0
  // 19.124: {31.000, 0.000, 3.000};   dt=2.000, dx/dt=0.5
  // 19.124: {32.000, 0.000, 3.000}
  /*std::vector<double> breaks = {0.000,  2.000,  4.000,  6.500, 9.012,
                                14.111, 16.624, 19.124, 21.124};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000, 0.000},  {0.000, 0.000, 0.000},  {1.000, 0.000, 0.000},
      {6.000, 0.000, 0.000},  {11.000, 0.000, 0.500}, {21.000,
  0.000, 2.500}, {26.000, 0.000, 3.000}, {31.000, 0.000, 3.000}, {32.000,
  0.000, 3.000}};
  cout << "Remember to increase the mid foot height to 0.15m\n";*/

  //////////////////////////////////////////////////////////////////////////////
  // Convert vector to MatrixXd
  std::vector<MatrixXd> knots(knots_vec.size(), MatrixX<double>(2, 1));
  for (int i = 0; i < knots_vec.size(); i++) {
    knots.at(i) << knots_vec.at(i)[0], knots_vec.at(i)[1];
  }

  // Construct the cubic splines for x y trajectory.
  desired_xy_traj_ = PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
  //  desired_xy_traj_ =
  //      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
  //          breaks, knots, MatrixX<double>::Zero(2, 1),
  //          MatrixX<double>::Zero(2, 1));

  // Construct the cubic splines for z trajectory.
  knots = std::vector<MatrixXd>(knots_vec.size(), MatrixX<double>::Zero(1, 1));
  if (knots_vec.at(0).size() > 2) {
    for (int i = 0; i < knots_vec.size(); i++) {
      knots.at(i) << knots_vec.at(i)[2];
    }
  }
  desired_z_traj_ = PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);

  /*for (int i = 0; i < breaks.size(); i++) {
    cout << breaks.at(i) << ", " << knots.at(i).transpose() << endl;
  }
  cout << "desired_xy_traj_ = \n";
  for (int i = 0; i < 60; i++) {
    cout << desired_xy_traj_.value(i * 0.1).transpose() << endl;
  }*/
};

EventStatus HighLevelCommand::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (high_level_mode_ == radio) {
    const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
        context, cassie_out_port_);
    // TODO(yangwill) make sure there is a message available
    // des_vel indices: 0: yaw_vel (right joystick left/right)
    //                  1: saggital_vel (left joystick up/down)
    //                  2: lateral_vel (left joystick left/right)
    Vector3d des_vel;
    des_vel << vel_scale_rot_ * cassie_out->pelvis.radio.channel[3],
        vel_scale_trans_sagital_ * cassie_out->pelvis.radio.channel[0],
        vel_scale_trans_lateral_ * cassie_out->pelvis.radio.channel[1];
    des_vel(1) += vel_command_offset_x_;  // hack: help rom_iter=300 walk in
                                          // place at start
    discrete_state->get_mutable_vector(des_vel_idx_).set_value(des_vel);
  } else if (high_level_mode_ == desired_xy_position) {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(CalcCommandFromTargetPosition(context));
  } else if (high_level_mode_ == open_loop_vel_command_traj) {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(desired_vel_command_traj_.value(context.get_time()));
  } else if (high_level_mode_ == desired_xy_traj) {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(CalcCommandFromDesiredXYTraj(context));
  }

  return EventStatus::Succeeded();
}

VectorXd HighLevelCommand::CalcCommandFromTargetPosition(
    const Context<double>& context) const {
  // Read in current state
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  VectorXd v = robotOutput->GetVelocities();

  plant_.SetPositions(context_, q);

  // Get center of mass position and velocity
  Vector3d com_pos = plant_.CalcCenterOfMassPositionInWorld(*context_);
  MatrixXd J(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, world_, world_, &J);
  Vector3d com_vel = J * v;

  //////////// Get desired yaw velocity ////////////
  // Get approximated heading angle of pelvis
  Vector3d pelvis_heading_vec =
      plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desired heading angle of pelvis
  Vector2d global_com_pos_to_target_pos =
      global_target_position_ - com_pos.segment(0, 2);
  double desired_yaw =
      atan2(global_com_pos_to_target_pos(1), global_com_pos_to_target_pos(0));

  // Get current yaw velocity
  double yaw_vel = v(2);

  // yaw error
  double heading_error =
      std::remainder(desired_yaw - approx_pelvis_yaw, 2 * M_PI);

  // PD position control
  double des_yaw_vel = kp_yaw_ * heading_error + kd_yaw_ * (-yaw_vel);
  des_yaw_vel = drake::math::saturate(des_yaw_vel, -vel_max_yaw_, vel_max_yaw_);
  /*cout << "desired_yaw= " << desired_yaw << endl;
  cout << "approx_pelvis_yaw= " << approx_pelvis_yaw << endl;
  cout << "heading_error= " << heading_error << endl;
  cout << "des_yaw_vel= " << des_yaw_vel << endl;
  cout << "\n";*/

  // Convex combination of 0 and desired yaw velocity
  double weight = 1 / (1 + exp(-params_of_no_turning_(0) *
                               (global_com_pos_to_target_pos.norm() -
                                params_of_no_turning_(1))));
  double desired_filtered_yaw_vel = (1 - weight) * 0 + weight * des_yaw_vel;

  //////////// Get desired horizontal vel ////////////
  // Calculate the current-desired yaw angle difference
  // filtered_heading_error is the convex combination of 0 and heading_error
  double filtered_heading_error = weight * heading_error;

  // Apply walking speed control only when the robot is facing the target
  // position.
  double des_sagital_vel = 0;
  double des_lateral_vel = 0;
  if (abs(filtered_heading_error) < M_PI / 2) {
    // Extract quaternion from floating base position
    Quaterniond Quat(q(0), q(1), q(2), q(3));
    Quaterniond Quat_conj = Quat.conjugate();
    Vector4d quat(q.head(4));
    Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(), Quat_conj.y(),
                       Quat_conj.z());

    // Calculate local target position and com velocity
    Vector3d global_com_pos_to_target_pos_3d;
    global_com_pos_to_target_pos_3d << global_com_pos_to_target_pos, 0;
    Vector3d local_com_pos_to_target_pos =
        drake::math::quatRotateVec(quad_conj, global_com_pos_to_target_pos_3d);
    Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

    // Sagital plane position PD control
    double com_vel_sagital = local_com_vel(0);
    des_sagital_vel = kp_pos_sagital_ * (local_com_pos_to_target_pos(0) +
                                         target_pos_offset_) +
                      kd_pos_sagital_ * (-com_vel_sagital);
    des_sagital_vel = drake::math::saturate(des_sagital_vel, -vel_max_sagital_,
                                            vel_max_sagital_);

    // Frontal plane position PD control.  TODO(yminchen): tune this
    double com_vel_lateral = local_com_vel(1);
    des_lateral_vel = kp_pos_lateral_ * (local_com_pos_to_target_pos(1)) +
                      kd_pos_lateral_ * (-com_vel_lateral);
    des_lateral_vel = drake::math::saturate(des_lateral_vel, -vel_max_lateral_,
                                            vel_max_lateral_);
  }
  Vector3d des_vel;
  des_vel << desired_filtered_yaw_vel, des_sagital_vel, des_lateral_vel;

  return des_vel;
}

VectorXd HighLevelCommand::CalcCommandFromDesiredXYTraj(
    const Context<double>& context) const {
  VectorXd q = dynamic_cast<const OutputVector<double>*>(
                   this->EvalVectorInput(context, state_port_))
                   ->GetPositions();
  VectorXd v = dynamic_cast<const OutputVector<double>*>(
                   this->EvalVectorInput(context, state_port_))
                   ->GetVelocities();
  Eigen::Matrix2d view_frame_rot_T_ =
      view_frame_->CalcWorldToFrameRotation(plant_, *context_)
          .topLeftCorner<2, 2>();
  Vector2d local_cur_xy_dot = view_frame_rot_T_ * v.segment<2>(3);

  // Advance the time for desired traj if the tracking error is not too big
  double dt_sim = (prev_t_ == 0) ? 0 : context.get_time() - prev_t_;
  if (!tracking_error_too_big_) {
    t_traj_ += dt_sim;
  }
  if (much_far_ahead_) {
    // We advance time faster than realtime if the current position is much
    // ahead of desired traj. This should be turn off if we want actual position
    // traj tracking.
    t_traj_ += dt_sim;
  }

  // Store dt's and compute the ratio of the traj time speed to sim time speed
  dt_buffer_sim_.at(dt_buffer_idx_) = dt_sim;
  dt_buffer_traj_.at(dt_buffer_idx_) = tracking_error_too_big_ ? 0 : dt_sim;
  dt_buffer_idx_++;
  if (dt_buffer_idx_ == dt_buffer_sim_.size()) {
    dt_buffer_idx_ = 0;
  }
  double time_rate_ratio_of_traj_to_sim =
      std::accumulate(dt_buffer_traj_.begin(), dt_buffer_traj_.end(), 0.0) /
      std::accumulate(dt_buffer_sim_.begin(), dt_buffer_sim_.end(), 0.0);

  bool reach_the_end_of_traj = desired_xy_traj_.end_time() <= t_traj_;
  bool close_to_the_end_of_traj = desired_xy_traj_.end_time() - 1.5 <= t_traj_;

  // Compute desired x y vel
  Vector2d des_xy = desired_xy_traj_.value(t_traj_);
  Vector2d cur_xy = q.segment<2>(4);
  Vector2d local_delta_xy = view_frame_rot_T_ * (des_xy - cur_xy);

  Vector2d des_xy_dot =
      desired_xy_traj_.has_derivative()
          ? desired_xy_traj_.EvalDerivative(t_traj_, 1)
          : desired_xy_traj_.MakeDerivative(1)->value(t_traj_);
  Vector2d local_des_xy_dot = view_frame_rot_T_ * des_xy_dot;

  if (close_to_the_end_of_traj) local_des_xy_dot.setZero();
  Vector2d command_xy_vel = 2 * local_delta_xy + local_des_xy_dot;

  // Compute yaw traj by taking derivaties of x y traj
  // yaw = arctan(dy,dx)
  Quaterniond cur_quat(q(0), q(1), q(2), q(3));
  Quaterniond des_quat =
      des_xy_dot.norm() < 0.01
          ? cur_quat
          : Quaterniond::FromTwoVectors(
                Vector3d(1, 0, 0), Vector3d(des_xy_dot(0), des_xy_dot(1), 0));
  Eigen::AngleAxis<double> angle_axis_err(des_quat * cur_quat.inverse());
  double yaw_err = (angle_axis_err.angle() * angle_axis_err.axis())(2);

  double delta_t =
      0.5;  // dt cannot be small, because our desired xy traj is not smooth
  Vector2d des_xy_dot_dt_later =
      desired_xy_traj_.has_derivative()
          ? desired_xy_traj_.EvalDerivative(t_traj_ + delta_t, 1)
          : desired_xy_traj_.MakeDerivative(1)->value(t_traj_ + delta_t);
  Quaterniond des_quat_dt_later =
      des_xy_dot_dt_later.norm() < 0.01
          ? cur_quat
          : Quaterniond::FromTwoVectors(
                Vector3d(1, 0, 0),
                Vector3d(des_xy_dot_dt_later(0), des_xy_dot_dt_later(1), 0));
  Eigen::AngleAxis<double> angle_axis_delta(des_quat_dt_later *
                                            des_quat.inverse());
  double delta_yaw = (angle_axis_delta.angle() * angle_axis_delta.axis())(2);
  double des_yaw_dot = time_rate_ratio_of_traj_to_sim * (delta_yaw / delta_t);

  if (reach_the_end_of_traj) des_yaw_dot = 0;
  double command_yaw_vel = 3 * yaw_err + des_yaw_dot;

  // Assign x, y and yaw vel
  Vector3d command_vel;
  // command_vel << command_yaw_vel, command_xy_vel(0), 0;

  // Instead of tracking des_vel_y here, we change the orientation to get closer
  // to the trajectory "center line"
  if ((std::abs(filtered_local_delta_y_) > 0.0) && !reach_the_end_of_traj) {
    command_yaw_vel += 2 * filtered_local_delta_y_;
  }
  command_vel << command_yaw_vel, command_xy_vel(0), 0;

  // Add low pass filter to the y deviation
  double alpha_y_deviation = 2 * M_PI * dt_sim * cutoff_freq_y_deviation_ /
                             (2 * M_PI * dt_sim * cutoff_freq_y_deviation_ + 1);
  filtered_local_delta_y_ = alpha_y_deviation * local_delta_xy(1) +
                            (1 - alpha_y_deviation) * filtered_local_delta_y_;

  // Add low pass filter to the command
  if (prev_t_ == 0) {
    filtered_vel_command_.setZero();
    //    filtered_vel_command_ = command_vel;
  } else {
    // cutoff_freq_ = (t_traj_ < 4) ? 0.1 : 1;
    if (reach_the_end_of_traj) {
      //      cutoff_freq_xy_ = 0.1;
      //      filtered_vel_command_.setZero();
    }

    double alpha_yaw = 2 * M_PI * dt_sim * cutoff_freq_yaw_ /
                       (2 * M_PI * dt_sim * cutoff_freq_yaw_ + 1);
    double alpha_xy = 2 * M_PI * dt_sim * cutoff_freq_xy_ /
                      (2 * M_PI * dt_sim * cutoff_freq_xy_ + 1);
    filtered_vel_command_.head<1>() =
        (alpha_yaw * command_vel + (1 - alpha_yaw) * filtered_vel_command_)
            .head<1>();
    filtered_vel_command_.tail<2>() =
        (alpha_xy * command_vel + (1 - alpha_xy) * filtered_vel_command_)
            .tail<2>();
  }

  // Update the flags
  double tol_lagging_behind_traj = 0.3;
  // Note that this parameter cannot be too big, it would affect des_yaw
  tracking_error_too_big_ = (local_delta_xy(0) > tol_lagging_behind_traj);
  //  tracking_error_too_big_ = false;

  much_far_ahead_ = (local_delta_xy(0) < -0.2);
  //  much_far_ahead_ = false;

  prev_t_ = context.get_time();

  /*cout << "t sime = " << context.get_time() << endl;
  cout << "t traj = " << t_traj_ << endl;
  cout << "des_xy = " << des_xy.transpose() << endl;
  cout << "local_delta_xy = " << local_delta_xy.transpose() << endl;
  cout << "local_des_xy_dot = " << local_des_xy_dot.transpose() << endl;

  cout << "des_xy_dot = " << des_xy_dot.transpose() << endl;
  cout << "des_xy_dot_dt_later = " << des_xy_dot_dt_later.transpose() << endl;
  cout << "---\n";
  cout << "cur_quat = " << cur_quat.w() << " " << cur_quat.vec().transpose()
       << endl;
  cout << "des_quat = " << des_quat.w() << " " << des_quat.vec().transpose()
       << endl;
  cout << "des_quat_dt_later = " << des_quat_dt_later.w() << " "
       << des_quat_dt_later.vec().transpose() << endl;
  cout << "angle_axis_err = " << angle_axis_err.angle() << ", "
       << angle_axis_err.axis().transpose() << endl;
  cout << "angle_axis_delta = " << angle_axis_delta.angle() << ", "
       << angle_axis_delta.axis().transpose() << endl;
  cout << "yaw_err = " << yaw_err << endl;
  cout << "des_yaw_dot = " << des_yaw_dot << endl;

  cout << "command_vel = " << command_vel.transpose() << endl;
  cout << "filtered_vel_command_ = " << filtered_vel_command_.transpose()
       << endl;
  cout << "time_ratio_of_traj_to_sim_ = " << time_ratio_of_traj_to_sim_ << endl;
  cout << "=============\n";*/

  // Update for z trajectory (I wrote it here just to quickly code this up)
  // The second term below adjusts the time to account for lagging behind traj
  // (rough estimate). I don't add this estimate for now, since I can see in
  // some situation they can be very inaccurate (maybe I can just bound them)
  // double t_now = t_traj_ - (local_delta_xy(0) / local_des_xy_dot(0));
  double t_now =
      t_traj_ - std::clamp(local_cur_xy_dot(0) == 0
                               ? 0
                               : local_delta_xy(0) / local_cur_xy_dot(0),
                           -0.3, 0.3);
  double duration = 0.35;  // Stride duration
  double t_start = std::max(t_now - duration / 2, desired_z_traj_.start_time());
  double t_end = std::min(t_now + duration / 2, desired_z_traj_.end_time());
  double dx = (view_frame_rot_T_ * (desired_xy_traj_.value(t_end) -
                                    desired_xy_traj_.value(t_start)))(0);
  feedforward_slope_ =
      dx == 0
          ? 0
          : (desired_z_traj_.value(t_end) - desired_z_traj_.value(t_start))(0) /
                dx;
  /*cout << "t_now = " << t_now << endl;
  cout << "t_traj_ = " << t_traj_ << endl;
  cout << "local_delta_xy = " << local_delta_xy.transpose() << endl;
  cout << "local_des_xy_dot = " << local_des_xy_dot.transpose() << endl;
  cout << "local_cur_xy_dot = " << local_cur_xy_dot.transpose() << endl;
  cout << "desired_z_traj_.start_time() = " << desired_z_traj_.start_time()
       << endl;
  cout << "desired_z_traj_.end_time() = " << desired_z_traj_.end_time() << endl;
  cout << "t_now = " << t_now << endl;
  cout << "t_start = " << t_start << endl;
  cout << "t_end = " << t_end << endl;
  cout << "feedforward_slope_ = " << feedforward_slope_ << endl;*/

  //  return command_vel;
  return filtered_vel_command_;
  //  return Vector3d(filtered_vel_command_(0), command_vel(1), 0);
}

void HighLevelCommand::CopyHeadingAngle(const Context<double>& context,
                                        BasicVector<double>* output) const {
  double desired_heading_pos =
      context.get_discrete_state(des_vel_idx_).get_value()(0);
  // Assign
  output->get_mutable_value() << desired_heading_pos;
}

void HighLevelCommand::CopyDesiredHorizontalVel(
    const Context<double>& context, BasicVector<double>* output) const {
  auto delta_CP_3D_global =
      context.get_discrete_state(des_vel_idx_).get_value().tail(2);

  // Assign
  output->get_mutable_value() = delta_CP_3D_global;
}

void HighLevelCommand::CopyFeedforwardSlope(const Context<double>& context,
                                            BasicVector<double>* output) const {
  // Assign
  output->get_mutable_value() << feedforward_slope_;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
