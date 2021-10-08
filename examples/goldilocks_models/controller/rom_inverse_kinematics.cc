#include "examples/goldilocks_models/controller/rom_inverse_kinematics.h"

#include "common/eigen_utils.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/visualization_utils.h"
#include "solvers/nonlinear_constraint.h"

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
using std::to_string;
using std::vector;

double INF = std::numeric_limits<double>::infinity();

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

  // Cannot use reference probably because you use Eigen's block operation to
  // pass y into the constructor
  const VectorXd y_;
};

RomInverseKinematics::RomInverseKinematics(
    const drake::multibody::MultibodyPlant<double>& plant_controls,
    const IKSetting& param, bool debug_mode)
    : plant_control_(plant_controls),
      param_(param),
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
  this->DeclareAbstractOutputPort("lcmt_saved_traj_ik",
                                  &RomInverseKinematics::CalcIK);

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
  // Testing
  if (counter_ > 0) return;
  counter_ += 1;

  // Get timestamp from context time (time from main thread's lcm)
  // double current_time = context.get_time();

  ///
  /// Read input port
  ///
  LcmTrajectory rom_traj(*(this->EvalInputValue<dairlib::lcmt_saved_traj>(
      context, rom_traj_lcm_port_)));

  // Some variables
  const auto& traj_names = rom_traj.GetTrajectoryNames();
  int n_mode = traj_names.size() - 2;
  int n_y = rom_traj.GetTrajectory(traj_names[0]).datatypes.size() / 2;
  const LcmTrajectory::Trajectory& stance_foot_vec =
      rom_traj.GetTrajectory("stance_foot");

  // Construct traj from lcm message
  /*
  const LcmTrajectory::Trajectory& traj0 =
      rom_traj.GetTrajectory(traj_names[0]);
  PiecewisePolynomial<double> pp_part =
      PiecewisePolynomial<double>::CubicHermite(
          traj0.time_vector, traj0.datapoints.topRows(n_y),
          traj0.datapoints.bottomRows(n_y));
  for (int mode = 1; mode < n_mode; ++mode) {
    const LcmTrajectory::Trajectory& traj_i =
        rom_traj.GetTrajectory(traj_names[mode]);
    pp_part.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        traj_i.time_vector, traj_i.datapoints.topRows(n_y),
        traj_i.datapoints.bottomRows(n_y)));
  }*/

  ///
  /// Solve IK
  ///
  auto start = std::chrono::high_resolution_clock::now();
  double total_solve_time = 0;

  // Parameters
  double eps = 1e-4;
  double mid_swing_height = 0.1;

  // Some variables used in the bounds
  Vector3d eps_vec = eps * VectorXd::Ones(3);
  //  Vector3d swing_foot_lb = -INF * Vector3d::Ones();
  //  Vector3d swing_foot_ub = INF * Vector3d::Ones();

  // Initialize IK solutions
  std::vector<MatrixXd> q_sol_all_modes;

  // Run through each mode and knot points
  // TODO: this could be sped up in various ways
  //  1. just solve for a short horizon starting from current time
  //  2. construct the MP once and replace parameters in each solve.
  for (int mode = 0; mode < n_mode; mode++) {
    const LcmTrajectory::Trajectory& traj_i =
        rom_traj.GetTrajectory(traj_names[mode]);
    int n_knots = traj_i.time_vector.size();

    const MatrixXd& x_FOMs = rom_traj.GetTrajectory("FOM").datapoints;
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

    // Get the swing foot positions
    Vector3d swing_foot_pos_start;
    plant_control_.CalcPointsPositions(*context_, toe_frame_swing,
                                       mid_contact_disp_, world_frame_,
                                       &swing_foot_pos_start);
    Vector3d swing_foot_pos_end;
    plant_control_.SetPositions(context_.get(), q_planner_end);
    plant_control_.CalcPointsPositions(*context_, toe_frame_swing,
                                       mid_contact_disp_, world_frame_,
                                       &swing_foot_pos_end);

    MatrixXd q_sol_per_mode(nq_, n_knots);
    q_sol_per_mode.leftCols<1>() = q_planner_start;
    q_sol_per_mode.rightCols<1>() = q_planner_end;
    for (int j = 1; j < n_knots - 1; j++) {
      /// Construct IK object
      drake::multibody::InverseKinematics ik(plant_control_);
      // Four bar linkage constraint (without spring)
      ik.get_mutable_prog()->AddLinearConstraint(
          (ik.q())(pos_map_.at("knee_left")) +
              (ik.q())(pos_map_.at("ankle_joint_left")) ==
          M_PI * 13 / 180.0);
      ik.get_mutable_prog()->AddLinearConstraint(
          (ik.q())(pos_map_.at("knee_right")) +
              (ik.q())(pos_map_.at("ankle_joint_right")) ==
          M_PI * 13 / 180.0);

      // Stance foot position
      // TODO: maybe I need to impose two contact constraints for stance foot
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

      // Pelvis position
      /*ik.AddPositionConstraint(pelvis_frame_, Vector3d(0, 0, 0), world_frame_,
                               pelvis_pos - eps * VectorXd::Ones(3),
                               pelvis_pos + eps * VectorXd::Ones(3));*/

      // Swing foot position
      // Let's the height be a concave parabola going though (0,0) and
      // (n_knots-1, 0). The trajectory is -C*x*(x-T) where
      //   T = n_knots-1
      //   C = 4/T^2 * mid_foot_height
      double T = n_knots - 1;
      Vector3d swing_toe_pos =
          swing_foot_pos_start +
          (swing_foot_pos_end - swing_foot_pos_start) * j / T;
      swing_toe_pos(2) = -4 / (T * T) * mid_swing_height * j * (j - T);
      ik.AddPositionConstraint(toe_frame_swing, mid_contact_disp_, world_frame_,
                               swing_toe_pos - eps_vec,
                               swing_toe_pos + eps_vec);

      // ROM mapping constraint
      auto kin_constraint = std::make_shared<IkKinematicsConstraint>(
          left_stance ? *rom_ : *mirrored_rom_, plant_control_,
          traj_i.datapoints.col(j).head(n_y));
      ik.get_mutable_prog()->AddConstraint(kin_constraint, ik.q());

      // Get the desired position
      VectorXd q_desired =
          q_planner_start + (q_planner_end - q_planner_start) * j / T;
      ik.get_mutable_prog()->AddQuadraticErrorCost(
          Eigen::MatrixXd::Identity(nq_, nq_), q_desired, ik.q());
      ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_desired);

      /// Solve
      //  if (debug_mode_) {
      if (true) {
        ik.get_mutable_prog()->SetSolverOption(
            drake::solvers::SnoptSolver::id(), "Print file", "../snopt_ik.out");
      }
      ik.get_mutable_prog()->SetSolverOption(
          drake::solvers::SnoptSolver::id(), "Major optimality tolerance",
          param_.feas_tol);  // target nonlinear constraint violation
      ik.get_mutable_prog()->SetSolverOption(
          drake::solvers::SnoptSolver::id(), "Major feasibility tolerance",
          param_.opt_tol);  // target complementarity gap
      // TODO: can I move SnoptSolver outside for loop?
      drake::solvers::SnoptSolver snopt_solver;
      auto start_inner = std::chrono::high_resolution_clock::now();
      const auto result =
          snopt_solver.Solve(ik.prog(), ik.prog().initial_guess());
      auto finish_inner = std::chrono::high_resolution_clock::now();

      SolutionResult solution_result = result.get_solution_result();
      cout << solution_result << " | ";
      cout << "Cost:" << result.get_optimal_cost() << "\n";

      std::chrono::duration<double> elapsed_inner = finish_inner - start_inner;
      total_solve_time += elapsed_inner.count();

      /// Get solution
      const auto q_sol = result.GetSolution(ik.q());
      VectorXd q_sol_normd(nq_);
      q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(nq_ - 4);
      q_sol_per_mode.col(j) = q_sol_normd;
    }
    q_sol_all_modes.push_back(q_sol_per_mode);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Construction time : " << elapsed.count() - total_solve_time << "\n";
  cout << "Solve time : " << total_solve_time << "\n";
  cout << "Construction time + Solve time: " << elapsed.count() << "\n";

  if (false) {
    //  if (debug_mode_) {

    // 0: plot solution. 1: plot desired position in IK
    const int PLOT_OPTION = 0;

    std::vector<MatrixXd> q_to_plot;
    if (PLOT_OPTION == 0) {
      q_to_plot = q_sol_all_modes;
    } else {
      // Get all desired positions used in the IK's
      std::vector<MatrixXd> q_desired_all_modes;
      for (int mode = 0; mode < n_mode; mode++) {
        const LcmTrajectory::Trajectory& traj_i =
            rom_traj.GetTrajectory(traj_names[mode]);
        int n_knots = traj_i.time_vector.size();
        const MatrixXd& x_FOMs = rom_traj.GetTrajectory("FOM").datapoints;
        VectorXd q_planner_start = x_FOMs.col(2 * mode).head(nq_);
        VectorXd q_planner_end = x_FOMs.col(2 * mode + 1).head(nq_);
        MatrixXd q_init_per_mode(nq_, n_knots);
        for (int j = 1; j < n_knots - 1; j++) {
          VectorXd q_desired =
              q_planner_start +
              (q_planner_end - q_planner_start) * j / (n_knots - 1);
          q_init_per_mode.col(j) = q_desired;
        }
        q_desired_all_modes.push_back(q_init_per_mode);
      }
      q_to_plot = q_desired_all_modes;
    }

    int knot_idx = 0;
    MatrixXd poses(nq_, 0);
    VectorXd alphas(0);
    for (int mode = 0; mode < n_mode; mode++) {
      int new_length = q_to_plot[mode].cols();
      poses.conservativeResize(poses.rows(), poses.cols() + new_length);
      alphas.conservativeResize(alphas.size() + new_length);

      poses.block(0, knot_idx, nq_, new_length) = q_to_plot[mode];
      alphas.segment(knot_idx, new_length) << 1,
          0.2 * VectorXd::Ones(new_length - 2), 1;

      knot_idx += new_length;
    }

    //    cout << "poses = " << poses << endl;
    cout << "poses = " << poses.col(0) << endl;
    cout << "alphas = " << alphas.transpose() << endl;

    ///
    multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        poses.cols(), alphas);
    visualizer.DrawPoses(poses);
  }

  ///
  /// Pack traj into lcm message (traj_msg)
  ///
  // Note that the trajectory is discontinuous between mode (even the position
  // jumps because left vs right stance leg).
  q_traj_msg->num_trajectories = n_mode;

  q_traj_msg->trajectory_names.resize(n_mode);
  q_traj_msg->trajectories.resize(n_mode);

  lcmt_trajectory_block traj_block;
  traj_block.num_datatypes = nq_;
  traj_block.datatypes.resize(nq_);
  traj_block.datatypes = vector<string>(nq_, "");
  for (int i = 0; i < n_mode; i++) {
    const LcmTrajectory::Trajectory& traj_i =
        rom_traj.GetTrajectory(traj_names[i]);
    int n_knots = traj_i.time_vector.size();

    /// Create lcmt_trajectory_block
    traj_block.trajectory_name = "";
    traj_block.num_points = n_knots;

    // Reserve space for vectors
    traj_block.time_vec.resize(traj_block.num_points);
    traj_block.datapoints.clear();

    // Copy Eigentypes to std::vector
    traj_block.time_vec = CopyVectorXdToStdVector(traj_i.time_vector);
    for (int j = 0; j < traj_block.num_datatypes; ++j) {
      traj_block.datapoints.push_back(
          CopyVectorXdToStdVector(q_sol_all_modes[i].row(j)));
    }

    /// Assign lcmt_trajectory_block
    q_traj_msg->trajectories[i] = traj_block;
    q_traj_msg->trajectory_names[i] = to_string(i);
  }
};

}  // namespace goldilocks_models
}  // namespace dairlib
