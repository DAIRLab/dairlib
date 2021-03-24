#include "rom_planner_saved_trajectory.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
using goldilocks_models::RomTrajOpt;

RomPlannerTrajectory::RomPlannerTrajectory(
    const RomTrajOpt& trajopt,
    const drake::solvers::MathematicalProgramResult& result,
    const VectorXd& quat_xyz_shift, const std::string& name,
    const std::string& description, bool lightweight) {
  num_modes_ = trajopt.num_modes();

  // Create state and input names
  const auto& rom = trajopt.reduced_order_model();
  std::vector<string> state_names;
  std::vector<string> input_names;
  if (lightweight) {
    state_names = std::vector<string>(rom.n_y() + rom.n_yddot(), "");
    input_names = std::vector<string>(rom.n_tau(), "");
  } else {
    for (int i = 0; i < rom.n_y(); i++) {
      state_names.push_back("y" + std::to_string(i));
    }
    for (int i = 0; i < rom.n_yddot(); i++) {
      state_names.push_back("ydot" + std::to_string(i));
    }
    for (int i = 0; i < rom.n_tau(); i++) {
      input_names.push_back("tau" + std::to_string(i));
    }
  }

  // State trajectory and force trajectory
  std::vector<Eigen::MatrixXd> x;
  std::vector<Eigen::MatrixXd> xdot;
  std::vector<Eigen::VectorXd> time_breaks;
  if (lightweight) {
    trajopt.GetStateSamples(result, &x, &time_breaks);
  } else {
    trajopt.GetStateAndDerivativeSamples(result, &x, &xdot, &time_breaks);
  }
  for (int mode = 0; mode < num_modes_; ++mode) {
    LcmTrajectory::Trajectory state_traj;
    LcmTrajectory::Trajectory state_derivative_traj;
    // LcmTrajectory::Trajectory force_traj;

    // State
    state_traj.traj_name = "state_traj" + std::to_string(mode);
    state_traj.datapoints = x[mode];
    state_traj.time_vector = time_breaks[mode];
    state_traj.datatypes = state_names;
    AddTrajectory(state_traj.traj_name, state_traj);

    if (!lightweight) {
      // State derivatives
      state_derivative_traj.traj_name =
          "state_derivative_traj" + std::to_string(mode);
      state_derivative_traj.datapoints = xdot[mode];
      state_derivative_traj.time_vector = time_breaks[mode];
      state_derivative_traj.datatypes = state_names;
      AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);

      /*// Force vars
      force_traj.traj_name = "force_vars" + std::to_string(mode);
      std::vector<std::string> force_names;
      std::vector<std::string> collocation_force_names;
      int num_forces = 0;
      for (int i = 0; i < trajopt.num_kinematic_constraints_wo_skipping(mode);
           ++i) {
        force_names.push_back("lambda_" + std::to_string(num_forces));
        collocation_force_names.push_back("lambda_c_" +
                                          std::to_string(num_forces));
        ++num_forces;
      }
      force_traj.traj_name = "force_vars" + std::to_string(mode);
      force_traj.time_vector = time_breaks[mode];
      force_traj.datapoints =
          Map<MatrixXd>(result.GetSolution(trajopt.force_vars(mode)).data(),
                        num_forces, force_traj.time_vector.size());
      force_traj.datatypes = force_names;
      // AddTrajectory(force_traj.traj_name, force_traj);

      // Collocation force vars
      if (time_breaks[mode].size() > 1) {
        LcmTrajectory::Trajectory collocation_force_traj;
        collocation_force_traj.traj_name =
            "collocation_force_vars" + std::to_string(mode);
        collocation_force_traj.datatypes = collocation_force_names;
        collocation_force_traj.time_vector =
            GetCollocationPoints(time_breaks[mode]);
        collocation_force_traj.datapoints = Map<MatrixXd>(
            result.GetSolution(trajopt.collocation_force_vars(mode)).data(),
            num_forces, collocation_force_traj.time_vector.size());
        AddTrajectory(collocation_force_traj.traj_name, collocation_force_traj);
        lambda_c_.push_back(&collocation_force_traj);
      }*/
    }

    // x_.push_back(&state_traj);
    // xdot_.push_back(&state_derivative_traj);
    // lambda_.push_back(&force_traj);
  }

  // Input trajectory
  if (!lightweight) {
    LcmTrajectory::Trajectory input_traj;
    input_traj.traj_name = "input_traj";
    input_traj.datapoints = trajopt.GetInputSamples(result);
    input_traj.time_vector = trajopt.GetSampleTimes(result);
    input_traj.datatypes = input_names;
    AddTrajectory(input_traj.traj_name, input_traj);
    // u_ = &input_traj;
  }

  // Decision variables
  if (!lightweight) {
    LcmTrajectory::Trajectory decision_var_traj;
    decision_var_traj.traj_name = "decision_vars";
    decision_var_traj.datapoints = result.GetSolution();
    decision_var_traj.time_vector = VectorXd::Zero(1);
    decision_var_traj.datatypes =
        vector<string>(decision_var_traj.datapoints.size());
    for (int i = 0; i < decision_var_traj.datapoints.size(); i++) {
      decision_var_traj.datatypes[i] = trajopt.decision_variable(i).get_name();
    }
    AddTrajectory(decision_var_traj.traj_name, decision_var_traj);
    // decision_vars_ = &decision_var_traj;
  }

  // Metadata
  if (!lightweight) {
    ConstructMetadataObject(name, description);
  } else {
    metadata_.description =
        drake::solvers::to_string(result.get_solution_result());
  }

  // 2. FOM x0 and xf
  MatrixXd x0_FOM = MatrixXd::Zero(trajopt.n_x_FOM(), num_modes_);
  MatrixXd xf_FOM = MatrixXd::Zero(trajopt.n_x_FOM(), num_modes_);
  for (int i = 0; i < num_modes_; ++i) {
    x0_FOM.col(i) = result.GetSolution(trajopt.x0_vars_by_mode(i));
    xf_FOM.col(i) = result.GetSolution(trajopt.xf_vars_by_mode(i));
  }
  VectorXd time_vec(num_modes_);
  for (int i = 0; i < num_modes_; i++) {
    time_vec(i) = time_breaks[i].head<1>()(0);
  }
  LcmTrajectory::Trajectory x0_traj;
  x0_traj.traj_name = "x0_FOM";
  x0_traj.datapoints = x0_FOM;
  x0_traj.time_vector = time_vec;
  x0_traj.datatypes = vector<string>(trajopt.n_x_FOM(), "");
  AddTrajectory(x0_traj.traj_name, x0_traj);
  for (int i = 0; i < num_modes_; i++) {
    time_vec(i) = time_breaks[i].tail<1>()(0);
  }
  LcmTrajectory::Trajectory xf_traj;
  xf_traj.traj_name = "xf_FOM";
  xf_traj.datapoints = xf_FOM;
  xf_traj.time_vector = time_vec;
  xf_traj.datatypes = vector<string>(trajopt.n_x_FOM(), "");
  AddTrajectory(xf_traj.traj_name, xf_traj);

  // 3. stance foot (left is 0, right is 1)
  MatrixXd stance_foot_mat = MatrixXd::Zero(1, num_modes_);
  for (int i = trajopt.start_with_left_stance() ? 1 : 0; i < num_modes_;
       i += 2) {
    stance_foot_mat(0, i) = 1;
  }
  LcmTrajectory::Trajectory stance_foot_traj;
  stance_foot_traj.traj_name = "stance_foot";
  stance_foot_traj.datapoints = stance_foot_mat;
  stance_foot_traj.time_vector = VectorXd::Zero(num_modes_);
  stance_foot_traj.datatypes = vector<string>(1, "");
  AddTrajectory(stance_foot_traj.traj_name, stance_foot_traj);

  // 4. x, y, z and yaw shift.
  LcmTrajectory::Trajectory quat_xyz_shift_traj;
  quat_xyz_shift_traj.traj_name = "quat_xyz_shift";
  quat_xyz_shift_traj.datapoints = quat_xyz_shift;
  quat_xyz_shift_traj.time_vector = VectorXd::Zero(1);
  quat_xyz_shift_traj.datatypes = {"quat_w", "quat_x", "quat_y", "quat_z",
                                   "x",      "y",      "z"};
  AddTrajectory(quat_xyz_shift_traj.traj_name, quat_xyz_shift_traj);
}

RomPlannerTrajectory::RomPlannerTrajectory(const lcmt_saved_traj& traj)
    : LcmTrajectory(traj) {
  // This is for the lightweight version. Used fro communicating between
  // processes online

  num_modes_ = 0;
  for (const auto& traj_name : GetTrajectoryNames()) {
    if (traj_name.find("state_traj") != std::string::npos) {
      ++num_modes_;
    }
  }

  // State
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(&GetTrajectory("state_traj" + std::to_string(mode)));
  }

  // x0_FOM, xf_FOM
  x0_FOM_ = &GetTrajectory("x0_FOM");
  xf_FOM_ = &GetTrajectory("xf_FOM");

  // stance_foot
  stance_foot_ = GetTrajectory("stance_foot").datapoints.row(0).transpose();

  // quat_xyz_shift
  quat_xyz_shift_ = GetTrajectory("quat_xyz_shift").datapoints.col(0);
}

void RomPlannerTrajectory::LoadFromFile(const std::string& filepath,
                                        bool lightweight) {
  LcmTrajectory::LoadFromFile(filepath);

  // Find all the state trajectories
  for (const auto& traj_name : GetTrajectoryNames()) {
    if (traj_name.find("state_traj") != std::string::npos) {
      ++num_modes_;
    }
  }

  // State and forces
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(&GetTrajectory("state_traj" + std::to_string(mode)));
    if (!lightweight) {
      xdot_.push_back(
          &GetTrajectory("state_derivative_traj" + std::to_string(mode)));
      /*lambda_.push_back(&GetTrajectory("force_vars" + std::to_string(mode)));
      if (x_[mode]->time_vector.size() > 1) {
        lambda_c_.push_back(
            &GetTrajectory("collocation_force_vars" + std::to_string(mode)));
      }*/
    }
  }

  // Input and all decision variables
  if (!lightweight) {
    u_ = &GetTrajectory("input_traj");
    decision_vars_ = &GetTrajectory("decision_vars");
  }

  // x0_FOM, xf_FOM
  x0_FOM_ = &GetTrajectory("x0_FOM");
  xf_FOM_ = &GetTrajectory("xf_FOM");

  // stance_foot
  stance_foot_ = GetTrajectory("stance_foot").datapoints.row(0).transpose();

  // quat_xyz_shift
  quat_xyz_shift_ = GetTrajectory("quat_xyz_shift").datapoints.col(0);
}

Eigen::VectorXd RomPlannerTrajectory::GetCollocationPoints(
    const Eigen::VectorXd& time_vector) {
  // using a + (b - a) / 2 midpoint
  int num_knotpoints = time_vector.size();
  return time_vector.head(num_knotpoints - 1) +
         0.5 * (time_vector.tail(num_knotpoints - 1) -
                time_vector.head(num_knotpoints - 1));
}

PiecewisePolynomial<double> RomPlannerTrajectory::ReconstructStateTrajectory()
    const {
  PiecewisePolynomial<double> state_traj =
      PiecewisePolynomial<double>::CubicHermite(
          x_[0]->time_vector, x_[0]->datapoints, xdot_[0]->datapoints);

  for (int mode = 1; mode < num_modes_; ++mode) {
    // Cannot form trajectory with only a single break
    if (x_[mode]->time_vector.size() < 2) {
      continue;
    }
    state_traj.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        x_[mode]->time_vector, x_[mode]->datapoints, xdot_[mode]->datapoints));
  }
  return state_traj;
}

PiecewisePolynomial<double> RomPlannerTrajectory::ReconstructInputTrajectory()
    const {
  PiecewisePolynomial<double> input_traj =
      PiecewisePolynomial<double>::FirstOrderHold(u_->time_vector,
                                                  u_->datapoints);

  return input_traj;
}

}  // namespace dairlib
