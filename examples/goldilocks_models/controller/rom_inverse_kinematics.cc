#include "examples/goldilocks_models/controller/rom_inverse_kinematics.h"

#include "multibody/multipose_visualizer.h"
#include "multibody/visualization_utils.h"
#include "solvers/nonlinear_constraint.h"

#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::string;

namespace dairlib {
namespace goldilocks_models {

class IkKinematicsConstraint : public solvers::NonlinearConstraint<double> {
 public:
  IkKinematicsConstraint(const ReducedOrderModel& rom,
                         const drake::multibody::MultibodyPlant<double>& plant,
                         const VectorXd& y, const std::string& description = "")
      : NonlinearConstraint<double>(rom.n_y(), plant.num_positions(),
                                    VectorXd::Zero(rom.n_y()),
                                    VectorXd::Zero(rom.n_y()), description),
        rom_(rom),
        plant_(plant),
        context_(plant.CreateDefaultContext()),
        y_(y){};

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& q,
                          drake::VectorX<double>* output) const override {
    // Update context
    plant_.SetPositions(context_.get(), q);
    // Impose constraint
    *output = y_ - rom_.EvalMappingFunc(q, *context_);
  };

  const ReducedOrderModel& rom_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const VectorXd& y_;
};

RomInverseKinematics::RomInverseKinematics(
    const drake::multibody::MultibodyPlant<double>& plant_controls,
    const IKSetting& param, bool debug_mode)
    : plant_control_(plant_controls),
      debug_mode_(debug_mode),
      nq_(plant_controls.num_positions()),
      nx_(plant_controls.num_positions() + plant_controls.num_velocities()),
      pos_map_(multibody::makeNameToPositionsMap(plant_controls)),
      context_(plant_controls.CreateDefaultContext()),
      world_frame_(plant_control_.world_frame()),
      pelvis_frame_(plant_control_.GetFrameByName("pelvis")),
      toe_left_frame_(plant_control_.GetFrameByName("toe_left")),
      toe_right_frame_(plant_control_.GetFrameByName("toe_right")),
      mid_contact_disp_((LeftToeFront(plant_controls).first +
                         LeftToeRear(plant_controls).first) /
                        2) {
  rom_traj_lcm_port_ =
      this->DeclareAbstractInputPort("rom_traj_lcm",
                                     drake::Value<dairlib::lcmt_saved_traj>{})
          .get_index();
  this->DeclareAbstractOutputPort(&RomInverseKinematics::CalcIK);

  // Reduced order model
  const int CASSIE_ID = 1;
  rom_ = CreateRom(param.rom_option, CASSIE_ID, plant_controls, false);
  ReadModelParameters(rom_.get(), param.dir_model, param.iter);
  // Create mirrorred reduced order model
  state_mirror_ =
      StateMirror(MirrorPosIndexMap(plant_controls, CASSIE_ID),
                  MirrorPosSignChangeSet(plant_controls, CASSIE_ID),
                  MirrorVelIndexMap(plant_controls, CASSIE_ID),
                  MirrorVelSignChangeSet(plant_controls, CASSIE_ID));
  mirrored_rom_ = std::make_unique<MirroredReducedOrderModel>(
      plant_controls, *rom_, state_mirror_);
};

void RomInverseKinematics::CalcIK(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_saved_traj* q_traj_msg) const {
  // Get timestamp from context time (time from main thread's lcm)
  double current_time = context.get_time();

  ///
  /// Read input port
  ///
  LcmTrajectory traj_data(*(this->EvalInputValue<dairlib::lcmt_saved_traj>(
      context, rom_traj_lcm_port_)));

  // Some variables
  const auto& traj_names = traj_data.GetTrajectoryNames();
  int n_mode = traj_names.size() - 2;
  int n_y = traj_data.GetTrajectory(traj_names[0]).datatypes.size() / 2;
  const LcmTrajectory::Trajectory& stance_foot_vec =
      traj_data.GetTrajectory("stance_foot");

  // Construct traj from lcm message
  /*
  const LcmTrajectory::Trajectory& traj0 =
      traj_data.GetTrajectory(traj_names[0]);
  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicHermite(
          traj0.time_vector, traj0.datapoints.topRows(n_y),
          traj0.datapoints.bottomRows(n_y));
  for (int mode = 1; mode < n_mode; ++mode) {
    const LcmTrajectory::Trajectory& traj_i =
        traj_data.GetTrajectory(traj_names[mode]);
    pp_part.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        traj_i.time_vector, traj_i.datapoints.topRows(n_y),
        traj_i.datapoints.bottomRows(n_y)));
  }*/

  ///
  /// Solve IK
  ///
  auto start = std::chrono::high_resolution_clock::now();

  // Parameters
  double eps = 1e-3;
  Vector3d eps_vec = eps * VectorXd::Ones(3);

  // Initialize IK solutions
  std::vector<MatrixXd> q_sol_all_modes;
  //  std::vector<MatrixXd> q_init_all_modes;

  // Run through each mode and knot points
  // TODO: this could be sped up in various ways
  //  1. just solve for a short horizon starting from current time
  //  2. construct the MP once and replace parameters in each solve.
  for (int mode = 0; mode < 1; mode++) {
    // for (int mode = 0; mode < n_mode; mode++) {
    const LcmTrajectory::Trajectory& traj_i =
        traj_data.GetTrajectory(traj_names[mode]);
    int n_knots = traj_i.time_vector.size();

    const MatrixXd& x_FOMs = traj_data.GetTrajectory("FOM").datapoints;
    VectorXd q_planner_start = x_FOMs.col(2 * mode).head(nq_);
    VectorXd q_planner_end = x_FOMs.col(2 * mode + 1).head(nq_);

    bool left_stance = (stance_foot_vec.datapoints(0, mode) == 0);
    const auto& toe_frame_stance =
        left_stance ? toe_left_frame_ : toe_right_frame_;
    const auto& toe_frame_swing =
        left_stance ? toe_right_frame_ : toe_left_frame_;
    string hip_yaw_stance = left_stance ? "hip_yaw_left" : "hip_yaw_right";
    string hip_yaw_swing = left_stance ? "hip_yaw_right" : "hip_yaw_left";

    // Get the stance foot position
    Vector3d stance_foot_pos;
    plant_control_.SetPositions(context_.get(), q_planner_start);
    plant_control_.CalcPointsPositions(*context_, toe_frame_stance,
                                       mid_contact_disp_, world_frame_,
                                       &stance_foot_pos);

    MatrixXd q_sol_per_mode(nq_, n_knots);
    q_sol_per_mode.leftCols<1>() = q_planner_start;
    q_sol_per_mode.rightCols<1>() = q_planner_end;
    //    MatrixXd q_init_per_mode = q_sol_per_mode;
    //    for (int j = 1; j < n_knots - 1; j++) {
    for (int j = 0; j < 1; j++) {
      /// Construct IK object
      drake::multibody::InverseKinematics ik(plant_control_);
      // Four bar linkage constraint (without spring)
      /*ik.get_mutable_prog()->AddLinearConstraint(
          (ik.q())(pos_map_.at("knee_left")) +
              (ik.q())(pos_map_.at("ankle_joint_left")) ==
          M_PI * 13 / 180.0);
      ik.get_mutable_prog()->AddLinearConstraint(
          (ik.q())(pos_map_.at("knee_right")) +
              (ik.q())(pos_map_.at("ankle_joint_right")) ==
          M_PI * 13 / 180.0);*/

      // Stance foot position
      ik.AddPositionConstraint(toe_frame_stance, mid_contact_disp_,
                               world_frame_, stance_foot_pos - eps_vec,
                               stance_foot_pos + eps_vec);
      // Pelvis pitch and roll
      // TODO: maybe we could use cost instead
      /*ik.get_mutable_prog()->AddBoundingBoxConstraint(
          -eps, eps, (ik.q())(pos_map_.at("base_qx")));
      ik.get_mutable_prog()->AddBoundingBoxConstraint(
          -eps, eps, (ik.q())(pos_map_.at("base_qy")));*/
      // Stance/swing hip yaw
      /*ik.get_mutable_prog()->AddBoundingBoxConstraint(
          0, 0, (ik.q())(pos_map_.at(hip_yaw_stance)));
      ik.get_mutable_prog()->AddBoundingBoxConstraint(
          0, 0, (ik.q())(pos_map_.at(hip_yaw_swing)));*/

      // Swing foot position
      /*ik.AddPositionConstraint(toe_frame_swing, mid_contact_disp_,
         world_frame_, swing_toe_pos - eps_vec, swing_toe_pos + eps_vec);*/
      // Pelvis position
      /*ik.AddPositionConstraint(pelvis_frame_, Vector3d(0, 0, 0), world_frame_,
                               pelvis_pos - eps * VectorXd::Ones(3),
                               pelvis_pos + eps * VectorXd::Ones(3));*/

      // ROM mapping constraint
      auto kin_constraint = std::make_shared<IkKinematicsConstraint>(
          left_stance ? *rom_ : *mirrored_rom_, plant_control_,
          traj_i.datapoints.col(j).head(n_y));
      ik.get_mutable_prog()->AddConstraint(kin_constraint, ik.q());

      // Get the desired position
      VectorXd q_desired = q_planner_start + (q_planner_end - q_planner_start) *
                                                 j / (n_knots - 1);
      ik.get_mutable_prog()->AddQuadraticErrorCost(
          Eigen::MatrixXd::Identity(nq_, nq_), q_desired, ik.q());
      ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_desired);

      /// Solve
      ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(),
                                             "Print file", "../snopt_ik.out");
      // TODO: can I move SnoptSolver outside for loop?
      drake::solvers::SnoptSolver snopt_solver;
      const auto result =
          snopt_solver.Solve(ik.prog(), ik.prog().initial_guess());

      SolutionResult solution_result = result.get_solution_result();
      cout << solution_result << " | ";
      cout << "Cost:" << result.get_optimal_cost() << "\n";

      /// Get solution
      const auto q_sol = result.GetSolution(ik.q());
      VectorXd q_sol_normd(nq_);
      q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(nq_ - 4);
      //      q_sol_normd << 1, 0, 0, 0, q_sol.tail(nq_ - 4);
      q_sol_per_mode.col(j) = q_sol_normd;

      //      q_init_per_mode.col(j) = q_desired;
    }
    q_sol_all_modes.push_back(q_sol_per_mode);
    //    q_init_all_modes.push_back(q_init_per_mode);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Solve time:" << elapsed.count() << "\n";

  if (true) {
    //  if (debug_mode_) {

    int knot_idx = 0;
    MatrixXd poses(nq_, 0);
    VectorXd alphas(0);
    /*for (int mode = 0; mode < n_mode; mode++) {
      int new_length = q_sol_all_modes[mode].cols();
      poses.conservativeResize(poses.rows(), poses.cols() + new_length);
      alphas.conservativeResize(alphas.size() + new_length);

      poses.block(0, knot_idx, nq_, new_length) = q_sol_all_modes[mode];
      alphas.segment(knot_idx, new_length) << 1,
          0.2 * VectorXd::Ones(new_length - 2), 1;

      knot_idx += new_length;
    }*/

    poses.conservativeResize(poses.rows(), 1);
    alphas.conservativeResize(1);
    poses = q_sol_all_modes[0].col(0);
    alphas << 1;
    cout << "poses = " << poses << endl;
    cout << "alphas = " << alphas << endl;

    ///
    multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        poses.cols(), alphas);
    visualizer.DrawPoses(poses);
  }

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  /*// Note that the trajectory is discontinuous between mode (even the position
  // jumps because left vs right stance leg).
  traj_msg->metadata.description = drake::solvers::to_string(solution_result);
  traj_msg->num_trajectories = param_.n_step;

  traj_msg->trajectory_names.resize(param_.n_step);
  traj_msg->trajectories.resize(param_.n_step);

  lcmt_trajectory_block traj_block;
  traj_block.num_datatypes = state_samples[0].rows();
  traj_block.datatypes.resize(traj_block.num_datatypes);
  traj_block.datatypes = vector<string>(traj_block.num_datatypes, "");
  for (int i = 0; i < param_.n_step; i++) {
    /// Create lcmt_trajectory_block
    traj_block.trajectory_name = "";
    traj_block.num_points = time_breaks[i].size();

    // Reserve space for vectors
    traj_block.time_vec.resize(traj_block.num_points);
    traj_block.datapoints.clear();

    // Copy Eigentypes to std::vector
    traj_block.time_vec = CopyVectorXdToStdVector(time_breaks[i]);
    for (int j = 0; j < traj_block.num_datatypes; ++j) {
      traj_block.datapoints.push_back(
          CopyVectorXdToStdVector(state_samples[i].row(j)));
    }

    /// Assign lcmt_trajectory_block
    traj_msg->trajectories[i] = traj_block;
    traj_msg->trajectory_names[i] = to_string(i);
  }*/
};

}  // namespace goldilocks_models
}  // namespace dairlib
