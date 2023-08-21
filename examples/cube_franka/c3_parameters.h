#pragma once

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
    a->Visit(DRAKE_NVP(translational_damping_ratio));
    a->Visit(DRAKE_NVP(rotational_damping));
    a->Visit(DRAKE_NVP(translational_integral_gain));
    a->Visit(DRAKE_NVP(rotational_integral_gain));
    a->Visit(DRAKE_NVP(integrator_clamp));
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
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(G));
    a->Visit(DRAKE_NVP(U_default));
    a->Visit(DRAKE_NVP(U_pos_vel));
    a->Visit(DRAKE_NVP(U_u));
    a->Visit(DRAKE_NVP(q_init_finger));
    a->Visit(DRAKE_NVP(q_init_ball_c3));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(velocity_limit));
    a->Visit(DRAKE_NVP(orientation_degrees));
    a->Visit(DRAKE_NVP(axis_option));

    // initialization/simulation parameters
    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_ball));
    a->Visit(DRAKE_NVP(initial_start));
    a->Visit(DRAKE_NVP(initial_finish));
    a->Visit(DRAKE_NVP(stabilize_time1));
    a->Visit(DRAKE_NVP(move_time));
    a->Visit(DRAKE_NVP(stabilize_time2));
    a->Visit(DRAKE_NVP(sim_dt));
    a->Visit(DRAKE_NVP(realtime_rate));

    // trajectory parameters
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(phase));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(degree_increment));
    a->Visit(DRAKE_NVP(time_increment));
    a->Visit(DRAKE_NVP(hold_order));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(enable_adaptive_path));
    
    // geometry parameters
    a->Visit(DRAKE_NVP(ball_radius));
    a->Visit(DRAKE_NVP(finger_radius));
    a->Visit(DRAKE_NVP(EE_offset));
    a->Visit(DRAKE_NVP(table_offset));
    a->Visit(DRAKE_NVP(model_table_offset));

    // misc
    a->Visit(DRAKE_NVP(contact_threshold));
    a->Visit(DRAKE_NVP(enable_heuristic));
    a->Visit(DRAKE_NVP(enable_contact));
    a->Visit(DRAKE_NVP(ball_stddev));

    // gaiting params
    a->Visit(DRAKE_NVP(roll_phase));
    a->Visit(DRAKE_NVP(return_phase));
    a->Visit(DRAKE_NVP(gait_parameters));

    // filter params
    a->Visit(DRAKE_NVP(dt_filter_length));
    a->Visit(DRAKE_NVP(alpha_p));
    a->Visit(DRAKE_NVP(alpha_v));

    // sampling params
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(sample_height));
    a->Visit(DRAKE_NVP(sample_number));
    a->Visit(DRAKE_NVP(spline_width));
    a->Visit(DRAKE_NVP(C3_failure));
    a->Visit(DRAKE_NVP(repositioning_threshold));
    a->Visit(DRAKE_NVP(travel_cost));
    a->Visit(DRAKE_NVP(travel_speed));
    
  }

  // impedance control parameters
  double translational_stiffness;
  double rotational_stiffness;
  double translational_damping_ratio;
  double rotational_damping;
  double translational_integral_gain;
  double rotational_integral_gain;
  VectorXd integrator_clamp;
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
  double R;
  double G;
  double U_default;
  double U_pos_vel;
  double U_u;
  VectorXd q_init_finger;
  VectorXd q_init_ball_c3;
  double dt;
  double velocity_limit;
  double orientation_degrees;
  int axis_option;

  // initialization/simulation parameters
  VectorXd q_init_franka;
  VectorXd q_init_ball;
  VectorXd initial_start;
  VectorXd initial_finish;
  double stabilize_time1;
  double move_time;
  double stabilize_time2;
  double sim_dt;
  double realtime_rate;

  // trajectory parameters
  double traj_radius;
  double phase;
  double x_c;
  double y_c;
  double degree_increment;
  double time_increment;
  int hold_order;

  // geometry parameters
  double ball_radius;
  double finger_radius;
  VectorXd EE_offset;
  double table_offset;
  double model_table_offset;

  //sampling parameters
  double sampling_radius;
  int sample_number;
  double sample_height;
  double spline_width;

  //switching parameters
  double C3_failure;
  double repositioning_threshold;
  double travel_cost;
  double travel_speed;

  // misc
  double contact_threshold;
  int enable_heuristic;
  int enable_contact;
  double ball_stddev;

  // gaiting parameters
  double roll_phase;
  double return_phase;
  VectorXd gait_parameters;
  
  // test parameters
  double lead_angle;
  int enable_adaptive_path;

  // filter parameters
  uint32_t dt_filter_length;
  double alpha_p;
  double alpha_v;

};