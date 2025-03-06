#include "mpc_reference.h"

namespace dairlib::systems::controllers::id_mpc {

lcmt_id_mpc_reference ConvertToLcm(const MPCReference& ref, double timestamp) {
  lcmt_id_mpc_reference msg;

  // Set timestamp
  msg.utime = 1e6 * timestamp;

  // Set knot times
  msg.num_knots = ref.knot_times_.size();
  msg.knot_times.resize(msg.num_knots);
  std::copy(ref.knot_times_.begin(), ref.knot_times_.end(), msg.knot_times.data());

  // Get dimensions from trajectories
  msg.num_q = ref.q_traj_.rows();
  msg.num_v = ref.v_traj_.rows();
  msg.num_u = ref.u_traj_.rows();
  msg.num_lambda = ref.lambda_traj_.rows();

  // Convert main trajectories
  // For each trajectory, we evaluate at the knot points and store in column-major format
  msg.q_traj.resize(msg.num_q, std::vector<double>(msg.num_knots));
  msg.quat_traj.resize(4, std::vector<double>(msg.num_knots));
  msg.v_traj.resize(msg.num_v, std::vector<double>(msg.num_knots));
  msg.lambda_traj.resize(msg.num_lambda, std::vector<double>(msg.num_knots));
  msg.u_traj.resize(msg.num_u, std::vector<double>(msg.num_knots));

  for (int i = 0; i < msg.num_knots; i++) {
    double t = msg.knot_times[i];

    // Evaluate each trajectory at the knot point
    Eigen::MatrixXd q_val = ref.q_traj_.value(t);
    Eigen::MatrixXd quat_val = ref.quat_traj_.value(t);
    Eigen::MatrixXd v_val = ref.v_traj_.value(t);
    Eigen::MatrixXd lambda_val = ref.lambda_traj_.value(t);
    Eigen::MatrixXd u_val = ref.u_traj_.value(t);

    // Copy values to message arrays
    for (int j = 0; j < msg.num_q; j++) {
      msg.q_traj[j][i] = q_val(j, 0);
    }
    for (int j = 0; j < 4; j++) {
      msg.quat_traj[j][i] = quat_val(j, 0);
    }
    for (int j = 0; j < msg.num_v; j++) {
      msg.v_traj[j][i] = v_val(j, 0);
    }
    for (int j = 0; j < msg.num_lambda; j++) {
      msg.lambda_traj[j][i] = lambda_val(j, 0);
    }
    for (int j = 0; j < msg.num_u; j++) {
      msg.u_traj[j][i] = u_val(j, 0);
    }
  }

  // Convert task space trajectories
  msg.num_task_space_trajs = ref.task_space_trajs_.size();
  msg.task_space_names.resize(msg.num_task_space_trajs);
  msg.task_space_dims.resize(msg.num_task_space_trajs);
  msg.task_space_trajs.resize(msg.num_task_space_trajs,
                              std::vector<double>(msg.num_knots));

  int traj_idx = 0;
  for (const auto& [name, traj] : ref.task_space_trajs_) {
    msg.task_space_names[traj_idx] = name;
    msg.task_space_dims[traj_idx] = traj.rows();

    for (int i = 0; i < msg.num_knots; i++) {
      Eigen::MatrixXd val = traj.value(msg.knot_times[i]);
      msg.task_space_trajs[traj_idx][i] = val(0, 0);
    }
    traj_idx++;
  }

  // Convert active contacts
  msg.max_contacts_per_knot = 0;
  for (const auto& contacts : ref.active_contacts_) {
    msg.max_contacts_per_knot = std::max(msg.max_contacts_per_knot,
                                         static_cast<int32_t>(contacts.size()));
  }

  msg.num_contacts_at_knot.resize(msg.num_knots);
  msg.active_contacts.resize(msg.num_knots,
                             std::vector<std::string>(msg.max_contacts_per_knot));

  for (int i = 0; i < msg.num_knots; i++) {
    msg.num_contacts_at_knot[i] = ref.active_contacts_[i].size();
    for (int j = 0; j < msg.num_contacts_at_knot[i]; j++) {
      msg.active_contacts[i][j] = ref.active_contacts_[i][j];
    }
  }

  // Convert touchdown information
  msg.num_touchdowns = ref.touchdown_ee_names_.size();
  msg.touchdown_ee_names = ref.touchdown_ee_names_;

  msg.touchdown_ee_points.resize(msg.num_touchdowns, std::vector<double>(3));
  for (int i = 0; i < msg.num_touchdowns; i++) {
    for (int j = 0; j < 3; j++) {
      msg.touchdown_ee_points[i][j] = ref.touchdown_ee_points_[i][j];
    }
  }

  // Convert touchdown updates
  msg.num_touchdown_updates = ref.touchdown_ee_names_to_update_.size();
  msg.touchdown_ee_names_to_update = ref.touchdown_ee_names_to_update_;

  return msg;
}

}