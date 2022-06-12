#include "drake/common/yaml/yaml_read_archive.h"
#include "yaml-cpp/yaml.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct C3Parameters {
  // must be serialized in same order as struct
  template <typename Archive>
  void Serialize(Archive* a) {
    // impedance control parameters
    a->Visit(DRAKE_NVP(translational_stiffness));
    a->Visit(DRAKE_NVP(rotational_stiffness));
    a->Visit(DRAKE_NVP(damping_ratio));
    a->Visit(DRAKE_NVP(stiffness_null));
    a->Visit(DRAKE_NVP(damping_null));
    a->Visit(DRAKE_NVP(q_null_desired));
    a->Visit(DRAKE_NVP(moving_offset));
    a->Visit(DRAKE_NVP(pushing_offset));

    // c3 parameters
    a->Visit(DRAKE_NVP(mu));
    a->Visit(DRAKE_NVP(Q_default));
    a->Visit(DRAKE_NVP(Q_finger));
    a->Visit(DRAKE_NVP(Q_ball_x));
    a->Visit(DRAKE_NVP(Q_ball_y));
    a->Visit(DRAKE_NVP(Q_finger_vel));
    a->Visit(DRAKE_NVP(Q_ball_vel));
    a->Visit(DRAKE_NVP(Qnew_finger));
    a->Visit(DRAKE_NVP(Qnew_ball_x));
    a->Visit(DRAKE_NVP(Qnew_ball_y));
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(G));
    a->Visit(DRAKE_NVP(U_default));
    a->Visit(DRAKE_NVP(U_pos_vel));
    a->Visit(DRAKE_NVP(U_u));
    a->Visit(DRAKE_NVP(q_init_finger));
    a->Visit(DRAKE_NVP(q_init_ball_c3));
    a->Visit(DRAKE_NVP(dt));

    // initialization/simulation parameters
    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_ball));
    a->Visit(DRAKE_NVP(initial_start));
    a->Visit(DRAKE_NVP(initial_finish));
    a->Visit(DRAKE_NVP(stabilize_time1));
    a->Visit(DRAKE_NVP(move_time));
    a->Visit(DRAKE_NVP(stabilize_time2));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(degree_increment));
    a->Visit(DRAKE_NVP(time_increment));
    a->Visit(DRAKE_NVP(hold_order));
    a->Visit(DRAKE_NVP(c3_sub_timeout));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(ball_radius));
    a->Visit(DRAKE_NVP(finger_radius));
    a->Visit(DRAKE_NVP(EE_offset));

    // misc
    a->Visit(DRAKE_NVP(contact_threshold));
    a->Visit(DRAKE_NVP(period));
    a->Visit(DRAKE_NVP(duty_cycle));
    a->Visit(DRAKE_NVP(enable_heuristic));
  }

  // impedance control parameters
  double translational_stiffness;
  double rotational_stiffness;
  double damping_ratio;
  double stiffness_null;
  double damping_null;
  VectorXd q_null_desired;
  double moving_offset;
  double pushing_offset;

  // c3 parameters
  double mu;
  double Q_default;
  double Q_finger;
  double Q_ball_x;
  double Q_ball_y;
  double Q_finger_vel;
  double Q_ball_vel;
  double Qnew_finger;
  double Qnew_ball_x;
  double Qnew_ball_y;
  double R;
  double G;
  double U_default;
  double U_pos_vel;
  double U_u;
  VectorXd q_init_finger;
  VectorXd q_init_ball_c3;
  double dt;

  // initialization/simulation parameters
  VectorXd q_init_franka;
  VectorXd q_init_ball;
  VectorXd initial_start;
  VectorXd initial_finish;
  double stabilize_time1;
  double move_time;
  double stabilize_time2;
  double traj_radius;
  double x_c;
  double y_c;
  double degree_increment;
  double time_increment;
  int hold_order;
  double c3_sub_timeout;
  double realtime_rate;
  double ball_radius;
  double finger_radius;
  VectorXd EE_offset;

  // misc
  double contact_threshold;
  double period;
  double duty_cycle;
  int enable_heuristic;
};