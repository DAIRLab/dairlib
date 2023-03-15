#include "hybrid_rom_planner_saved_trajectory.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

namespace dairlib {
using goldilocks_models::HybridRomTrajOpt;

// Assignment operator (for the lightweight version!)
// Reminder: anything that you want to send to the controller thread should be
// copied here.
// The copy assignment is used in cassie_rom_planner_system.cc like
//  lightweight_saved_traj_ =
//      HybridRomPlannerTrajectory(trajopt, result, global_x0_FOM_,
//      global_xf_FOM_, prefix, "", true, current_time);
HybridRomPlannerTrajectory& HybridRomPlannerTrajectory::operator=(
    const HybridRomPlannerTrajectory& old) {
  // Base class (LcmTrajectory)
  LcmTrajectory::operator=(old);

  // Derived class (HybridRomPlannerTrajectory)
  utime_ = old.get_utime();
  num_modes_ = old.GetNumModes();

  x_.clear();
  xdot_.clear();
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(&GetTrajectory("state_traj" + std::to_string(mode)));
    xdot_.push_back(
        &GetTrajectory("state_derivative_traj" + std::to_string(mode)));
  }

  global_feet_pos_ = &GetTrajectory("global_feet_pos");
  global_com_pos_ = &GetTrajectory("global_com_pos");

  //  stance_foot_ = old.get_stance_foot();

  // We do not copy the following data, because this copy assignment is only
  // used for the lightweight version
  //   u_ = &GetTrajectory("input_traj");
  //   decision_vars_;
  //   lambda_;
  //   lambda_c_;
  //  current_quat_xyz_shift_ = old.get_current_quat_xyz_shift();
  //  current_stance_foot_pos_ = old.get_current_stance_foot_pos();

  return *this;
}

// Even though we don't send xdot to the controller thread, we still save it in
// the lightweight version, becasue we use xdot in warmstarting
HybridRomPlannerTrajectory::HybridRomPlannerTrajectory(
    const HybridRomTrajOpt& trajopt,
    const drake::solvers::MathematicalProgramResult& result,
    const Eigen::MatrixXd& global_feet_pos,
    const Eigen::MatrixXd& global_com_pos,
    const Eigen::VectorXd& current_quat_xyz_shift,
    const Eigen::VectorXd& current_stance_foot_pos, const std::string& name,
    const std::string& description, bool lightweight, double current_time)
    : LcmTrajectory() {
  utime_ = int((current_time + 1e-8) * 1e6);
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

  std::vector<Eigen::MatrixXd> x;
  std::vector<Eigen::MatrixXd> xdot;
  std::vector<Eigen::VectorXd> time_breaks;
  //  if (lightweight) {
  //    trajopt.GetStateSamples(result, &x, &time_breaks);
  //  } else {
  trajopt.GetStateAndDerivativeSamples(result, &x, &xdot, &time_breaks);
  //  }

  // Shift the timestamps by the current time
  for (auto& time_break_per_mode : time_breaks) {
    time_break_per_mode.array() += current_time;
  }

  // State trajectory and force trajectory
  for (int mode = 0; mode < num_modes_; ++mode) {
    // State
    LcmTrajectory::Trajectory state_traj;
    state_traj.traj_name = "state_traj" + std::to_string(mode);
    state_traj.datapoints = x[mode];
    state_traj.time_vector = time_breaks[mode];
    state_traj.datatypes = state_names;
    AddTrajectory(state_traj.traj_name, state_traj);
    x_.push_back(&GetTrajectory(state_traj.traj_name));

    // State derivatives
    //    if (!lightweight) {
    LcmTrajectory::Trajectory state_derivative_traj;
    state_derivative_traj.traj_name =
        "state_derivative_traj" + std::to_string(mode);
    state_derivative_traj.datapoints = xdot[mode];
    state_derivative_traj.time_vector = time_breaks[mode];
    state_derivative_traj.datatypes = state_names;
    AddTrajectory(state_derivative_traj.traj_name, state_derivative_traj);
    xdot_.push_back(&GetTrajectory(state_derivative_traj.traj_name));
    //    }
  }  // end for

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
    // `ConstructMetadataObject` is very expensive.
    // It creates new thread to call git diff and git rev.
    // Since I don't use them, I just removed the git calls in the method.
    ConstructMetadataObject(name, description);
  } else {
    metadata_.name = name;
    metadata_.description =
        drake::solvers::to_string(result.get_solution_result());
  }

  // 2.a global feet pos
  VectorXd time_vec_discrete_event(num_modes_ + 1);
  for (int i = 0; i < num_modes_; i++) {
    time_vec_discrete_event(i) = time_breaks[i].head<1>()(0);
  }
  time_vec_discrete_event(num_modes_) =
      time_breaks[num_modes_ - 1].tail<1>()(0);
  LcmTrajectory::Trajectory global_feet_pos_traj;
  global_feet_pos_traj.traj_name = "global_feet_pos";
  global_feet_pos_traj.datapoints = global_feet_pos;
  global_feet_pos_traj.time_vector = time_vec_discrete_event;
  global_feet_pos_traj.datatypes = vector<string>(2, "");
  AddTrajectory(global_feet_pos_traj.traj_name, global_feet_pos_traj);
  global_feet_pos_ = &GetTrajectory(global_feet_pos_traj.traj_name);
  // 2.b global com
  LcmTrajectory::Trajectory global_com_pos_traj;
  global_com_pos_traj.traj_name = "global_com_pos";
  global_com_pos_traj.datapoints = global_com_pos;
  global_com_pos_traj.time_vector = time_vec_discrete_event;
  global_com_pos_traj.datatypes = vector<string>(2, "");
  AddTrajectory(global_com_pos_traj.traj_name, global_com_pos_traj);
  global_com_pos_ = &GetTrajectory(global_com_pos_traj.traj_name);

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

  // 4. current_quat_xyz_shift_
  LcmTrajectory::Trajectory quat_xyz_shift_traj;
  quat_xyz_shift_traj.traj_name = "current_quat_xyz_shift";
  quat_xyz_shift_traj.datapoints = current_quat_xyz_shift;
  quat_xyz_shift_traj.time_vector = VectorXd::Zero(1);
  quat_xyz_shift_traj.datatypes = vector<string>(7, "");
  AddTrajectory(quat_xyz_shift_traj.traj_name, quat_xyz_shift_traj);

  if (!lightweight) {
    // 5. current_stance_foot_pos_
    LcmTrajectory::Trajectory stance_foot_pos_traj;
    stance_foot_pos_traj.traj_name = "current_stance_foot_pos";
    stance_foot_pos_traj.datapoints = current_stance_foot_pos;
    stance_foot_pos_traj.time_vector = VectorXd::Zero(1);
    stance_foot_pos_traj.datatypes = vector<string>(3, "");
    AddTrajectory(stance_foot_pos_traj.traj_name, stance_foot_pos_traj);
  }
}

HybridRomPlannerTrajectory::HybridRomPlannerTrajectory(
    const lcmt_timestamped_saved_traj& traj)
    : LcmTrajectory(traj.saved_traj) {
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

  // global_feet_pos_
  global_feet_pos_ = &GetTrajectory("global_feet_pos");
  global_com_pos_ = &GetTrajectory("global_com_pos");

  // stance_foot
  //  stance_foot_ = GetTrajectory("stance_foot").datapoints.row(0).transpose();

  //  current_quat_xyz_shift_ =
  //  GetTrajectory("current_quat_xyz_shift").datapoints.row(0).transpose();
  //  current_stance_foot_pos_ =
  //  GetTrajectory("current_stance_foot_pos").datapoints.row(0).transpose();
}

lcmt_timestamped_saved_traj HybridRomPlannerTrajectory::GenerateLcmObject()
    const {
  lcmt_timestamped_saved_traj traj;
  traj.utime = utime_;
  traj.saved_traj = LcmTrajectory::GenerateLcmObject();

  return traj;
}

void HybridRomPlannerTrajectory::LoadFromFile(const std::string& filepath,
                                              bool lightweight) {
  LcmTrajectory::LoadFromFile(filepath);

  // Find all the state trajectories
  for (const auto& traj_name : GetTrajectoryNames()) {
    if (traj_name.find("state_traj") != std::string::npos) {
      ++num_modes_;
    }
  }

  // State
  for (int mode = 0; mode < num_modes_; ++mode) {
    x_.push_back(&GetTrajectory("state_traj" + std::to_string(mode)));
    //    if (!lightweight) {
    xdot_.push_back(
        &GetTrajectory("state_derivative_traj" + std::to_string(mode)));
    //    }
  }

  // Input and all decision variables
  if (!lightweight) {
    u_ = &GetTrajectory("input_traj");
    decision_vars_ = &GetTrajectory("decision_vars");
  }

  // global_feet_pos
  global_feet_pos_ = &GetTrajectory("global_feet_pos");
  global_com_pos_ = &GetTrajectory("global_com_pos");

  //  // stance_foot
  //  stance_foot_ = GetTrajectory("stance_foot").datapoints.row(0).transpose();
  //
  //  if (!lightweight) {
  //    current_quat_xyz_shift_ =
  //        GetTrajectory("current_quat_xyz_shift").datapoints.row(0).transpose();
  //    current_stance_foot_pos_ =
  //        GetTrajectory("current_stance_foot_pos").datapoints.row(0).transpose();
  //  }
}

Eigen::VectorXd HybridRomPlannerTrajectory::GetCollocationPoints(
    const Eigen::VectorXd& time_vector) {
  // using a + (b - a) / 2 midpoint
  int num_knotpoints = time_vector.size();
  return time_vector.head(num_knotpoints - 1) +
         0.5 * (time_vector.tail(num_knotpoints - 1) -
                time_vector.head(num_knotpoints - 1));
}

PiecewisePolynomial<double>
HybridRomPlannerTrajectory::ConstructPositionTrajectory() const {
  int n_y = x_[0]->datatypes.size() / 2;

  PiecewisePolynomial<double> pp;
  for (int mode = 0; mode < num_modes_; ++mode) {
    const LcmTrajectory::Trajectory* traj_i = x_[mode];
    pp.ConcatenateInTime(PiecewisePolynomial<double>::CubicHermite(
        traj_i->time_vector, traj_i->datapoints.topRows(n_y),
        traj_i->datapoints.bottomRows(n_y)));
  }
  return pp;
}

PiecewisePolynomial<double>
HybridRomPlannerTrajectory::ReconstructStateTrajectory() const {
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

PiecewisePolynomial<double>
HybridRomPlannerTrajectory::ReconstructInputTrajectory() const {
  PiecewisePolynomial<double> input_traj =
      PiecewisePolynomial<double>::FirstOrderHold(u_->time_vector,
                                                  u_->datapoints);

  return input_traj;
}

/// ReadHybridRomPlannerTrajectory
PiecewisePolynomial<double> ReadHybridRomPlannerTrajectory(
    const std::string& path, bool offset_time_to_zero) {
  HybridRomPlannerTrajectory traj_obj(path);
  auto ret = traj_obj.ReconstructStateTrajectory();

  if (offset_time_to_zero) {
    //    ret.shiftRight(-ret.start_time());

    // Testing
    ret.shiftRight(-traj_obj.get_global_feet_pos_time()(0));
    //    ret.shiftRight(-traj_obj.get_x0_time()(1));
  }

  return ret;
}

}  // namespace dairlib
