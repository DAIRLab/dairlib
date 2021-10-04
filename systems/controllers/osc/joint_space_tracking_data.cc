
/**** JointSpaceTrackingData ****/
JointSpaceTrackingData::JointSpaceTrackingData(
    const string& name, const MatrixXd& K_p, const MatrixXd& K_d,
    const MatrixXd& W, const MultibodyPlant<double>& plant_w_spr,
    const MultibodyPlant<double>& plant_wo_spr)
    : OscTrackingData(name, K_p.rows(), K_p.rows(), K_p, K_d, W, plant_w_spr,
                      plant_wo_spr) {}

void JointSpaceTrackingData::AddJointToTrack(
    const std::string& joint_pos_name, const std::string& joint_vel_name) {
  joint_pos_idx_w_spr_.push_back(
      {makeNameToPositionsMap(plant_w_spr_).at(joint_pos_name)});
  joint_vel_idx_w_spr_.push_back(
      {makeNameToVelocitiesMap(plant_w_spr_).at(joint_vel_name)});
  joint_pos_idx_wo_spr_.push_back(
      {makeNameToPositionsMap(plant_wo_spr_).at(joint_pos_name)});
  joint_vel_idx_wo_spr_.push_back(
      {makeNameToVelocitiesMap(plant_wo_spr_).at(joint_vel_name)});
}

void JointSpaceTrackingData::AddStateAndJointToTrack(
    int state, const std::string& joint_pos_name,
    const std::string& joint_vel_name) {
  AddState(state);
  AddJointToTrack(joint_pos_name, joint_vel_name);
}

void JointSpaceTrackingData::AddJointsToTrack(
    const std::vector<std::string>& joint_pos_names,
    const std::vector<std::string>& joint_vel_names) {
  std::vector<int> ordered_index_set;
  for (const auto& mem : joint_pos_names) {
    ordered_index_set.push_back(makeNameToPositionsMap(plant_w_spr_).at(mem));
  }
  joint_pos_idx_w_spr_.push_back(ordered_index_set);
  ordered_index_set.clear();
  for (const auto& mem : joint_vel_names) {
    ordered_index_set.push_back(makeNameToVelocitiesMap(plant_w_spr_).at(mem));
  }
  joint_vel_idx_w_spr_.push_back(ordered_index_set);
  ordered_index_set.clear();
  for (const auto& mem : joint_pos_names) {
    ordered_index_set.push_back(makeNameToPositionsMap(plant_wo_spr_).at(mem));
  }
  joint_pos_idx_wo_spr_.push_back(ordered_index_set);
  ordered_index_set.clear();
  for (const auto& mem : joint_vel_names) {
    ordered_index_set.push_back(makeNameToVelocitiesMap(plant_wo_spr_).at(mem));
  }
  joint_vel_idx_wo_spr_.push_back(ordered_index_set);
}

void JointSpaceTrackingData::AddStateAndJointsToTrack(
    int state, const std::vector<std::string>& joint_pos_names,
    const std::vector<std::string>& joint_vel_names) {
  AddState(state);
  AddJointsToTrack(joint_pos_names, joint_vel_names);
}

void JointSpaceTrackingData::UpdateYddotDes() {
  yddot_des_converted_ = yddot_des_;
}

void JointSpaceTrackingData::UpdateY(const VectorXd& x_w_spr,
                                     const Context<double>& context_w_spr) {
  VectorXd y(GetYDim());
  for (int i = 0; i < GetYDim(); i++) {
    y(i) = x_w_spr(joint_pos_idx_w_spr_.at(GetStateIdx()).at(i));
  }
  y_ = y;
}

void JointSpaceTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void JointSpaceTrackingData::UpdateYdot(const VectorXd& x_w_spr,
                                        const Context<double>& context_w_spr) {
  VectorXd ydot(GetYdotDim());
  for (int i = 0; i < GetYdotDim(); i++) {
    ydot(i) = x_w_spr(plant_w_spr_.num_positions() +
        joint_pos_idx_w_spr_.at(GetStateIdx()).at(i));
  }
  ydot_ = ydot;
}

void JointSpaceTrackingData::UpdateYdotError() {
  error_ydot_ = ydot_des_ - ydot_;
}

void JointSpaceTrackingData::UpdateJ(const VectorXd& x_wo_spr,
                                     const Context<double>& context_wo_spr) {
  MatrixXd J = MatrixXd::Zero(GetYdotDim(), plant_wo_spr_.num_velocities());
  for (int i = 0; i < GetYdotDim(); i++) {
    J(i, joint_vel_idx_wo_spr_.at(GetStateIdx()).at(i)) = 1;
  }
  J_ = J;
}

void JointSpaceTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = VectorXd::Zero(GetYdotDim());
}

void JointSpaceTrackingData::CheckDerivedOscTrackingData() {
  for (int i = 0; i < joint_pos_idx_w_spr_.size(); i++) {
    DRAKE_DEMAND(joint_pos_idx_w_spr_.at(i).size() == GetYDim());
    DRAKE_DEMAND(joint_pos_idx_wo_spr_.at(i).size() == GetYDim());
    DRAKE_DEMAND(joint_vel_idx_w_spr_.at(i).size() == GetYdotDim());
    DRAKE_DEMAND(joint_vel_idx_wo_spr_.at(i).size() == GetYdotDim());
  }
  if (state_.empty()) {
    DRAKE_DEMAND(joint_pos_idx_w_spr_.size() == 1);
    DRAKE_DEMAND(joint_vel_idx_w_spr_.size() == 1);
    DRAKE_DEMAND(joint_pos_idx_wo_spr_.size() == 1);
    DRAKE_DEMAND(joint_vel_idx_wo_spr_.size() == 1);
  } else {
    DRAKE_DEMAND(joint_pos_idx_w_spr_.size() == state_.size());
    DRAKE_DEMAND(joint_vel_idx_w_spr_.size() == state_.size());
    DRAKE_DEMAND(joint_pos_idx_wo_spr_.size() == state_.size());
    DRAKE_DEMAND(joint_vel_idx_wo_spr_.size() == state_.size());
  }
}
