

void RelativeTranslationTrackingData::PreUpdate(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr,
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr,
    const drake::trajectories::Trajectory<double>& traj, double t,
    double t_since_last_state_switch, int finite_state_machine_state,
    const Eigen::VectorXd& v_proj, bool no_desired_traj) {
  to_frame_data_.Update(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr, traj,
                        t, t_since_last_state_switch,
                        finite_state_machine_state, v_proj, true);
  from_frame_data_.Update(x_w_spr, context_w_spr, x_wo_spr, context_wo_spr,
                          traj, t, t_since_last_state_switch,
                          finite_state_machine_state, v_proj, true);
}

void RelativeTranslationTrackingData::UpdateYddotDes() {
  yddot_des_converted_ = yddot_des_;
}

void RelativeTranslationTrackingData::UpdateY(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  y_ = to_frame_data_.GetY() - from_frame_data_.GetY();
}

void RelativeTranslationTrackingData::UpdateYError() { error_y_ = y_des_ - y_; }

void RelativeTranslationTrackingData::UpdateYdot(
    const VectorXd& x_w_spr, const Context<double>& context_w_spr) {
  ydot_ = to_frame_data_.GetYdot() - from_frame_data_.GetYdot();
}

void RelativeTranslationTrackingData::UpdateYdotError() {
  error_ydot_ = ydot_des_ - ydot_;
}

void RelativeTranslationTrackingData::UpdateJ(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  J_ = to_frame_data_.GetJ() - from_frame_data_.GetJ();
}

void RelativeTranslationTrackingData::UpdateJdotV(
    const VectorXd& x_wo_spr, const Context<double>& context_wo_spr) {
  JdotV_ = to_frame_data_.GetJdotTimesV() - from_frame_data_.GetJdotTimesV();
}

void RelativeTranslationTrackingData::CheckDerivedOscTrackingData() {
  to_frame_data_.CheckOscTrackingData(true);
  from_frame_data_.CheckOscTrackingData(true);
}
