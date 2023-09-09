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
    a->Visit(DRAKE_NVP(use_full_cost));
    a->Visit(DRAKE_NVP(Q_default));
    a->Visit(DRAKE_NVP(Q_finger));
    a->Visit(DRAKE_NVP(Q_ball_x));
    a->Visit(DRAKE_NVP(Q_ball_y));
    a->Visit(DRAKE_NVP(Q_ball_z));
    a->Visit(DRAKE_NVP(Q_finger_vel));
    a->Visit(DRAKE_NVP(Q_ball_vel));
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(G));
    a->Visit(DRAKE_NVP(G_ground));
    a->Visit(DRAKE_NVP(U_default));
    a->Visit(DRAKE_NVP(U_pos));
    a->Visit(DRAKE_NVP(U_vel));
    a->Visit(DRAKE_NVP(U_u));
    a->Visit(DRAKE_NVP(q_init_finger));
    a->Visit(DRAKE_NVP(q_init_ball_c3));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(velocity_limit));
    a->Visit(DRAKE_NVP(orientation_degrees));
    a->Visit(DRAKE_NVP(axis_option));
    a->Visit(DRAKE_NVP(planning_timestep));
    a->Visit(DRAKE_NVP(c3_planned_next_state_timestep));
    a->Visit(DRAKE_NVP(horizon_length));

    // initialization/simulation parameters
    a->Visit(DRAKE_NVP(q_init_franka));
    a->Visit(DRAKE_NVP(q_init_ball));
    a->Visit(DRAKE_NVP(initial_start));
    a->Visit(DRAKE_NVP(initial_finish));
    a->Visit(DRAKE_NVP(stabilize_time1));
    a->Visit(DRAKE_NVP(move_time));
    a->Visit(DRAKE_NVP(stabilize_time2));
    a->Visit(DRAKE_NVP(sim_dt));
    a->Visit(DRAKE_NVP(sim_publish_dt));
    a->Visit(DRAKE_NVP(realtime_rate));

    // trajectory parameters
    a->Visit(DRAKE_NVP(trajectory_type));
    a->Visit(DRAKE_NVP(traj_radius));
    a->Visit(DRAKE_NVP(phase));
    a->Visit(DRAKE_NVP(x_c));
    a->Visit(DRAKE_NVP(y_c));
    a->Visit(DRAKE_NVP(degree_increment));
    a->Visit(DRAKE_NVP(time_increment));
    a->Visit(DRAKE_NVP(hold_order));
    a->Visit(DRAKE_NVP(lead_angle));
    a->Visit(DRAKE_NVP(fixed_goal_x));
    a->Visit(DRAKE_NVP(fixed_goal_y));
    
    // geometry parameters
    a->Visit(DRAKE_NVP(jack_half_width));
    a->Visit(DRAKE_NVP(finger_radius));
    a->Visit(DRAKE_NVP(EE_offset));
    a->Visit(DRAKE_NVP(table_offset));
    a->Visit(DRAKE_NVP(model_table_offset));

    // misc
    a->Visit(DRAKE_NVP(contact_threshold));
    a->Visit(DRAKE_NVP(enable_heuristic));
    a->Visit(DRAKE_NVP(enable_contact));
    a->Visit(DRAKE_NVP(ball_stddev));

    //Visualization
    a->Visit(DRAKE_NVP(display_rollout_flag));

    // filter params
    a->Visit(DRAKE_NVP(dt_filter_length));
    a->Visit(DRAKE_NVP(alpha_p));
    a->Visit(DRAKE_NVP(alpha_v));

    // sampling params
    a->Visit(DRAKE_NVP(force_skip_sampling));
    a->Visit(DRAKE_NVP(sampling_strategy));
    a->Visit(DRAKE_NVP(sampling_radius));
    a->Visit(DRAKE_NVP(sample_height));
    a->Visit(DRAKE_NVP(min_angle_from_vertical));
    a->Visit(DRAKE_NVP(max_angle_from_vertical));
    a->Visit(DRAKE_NVP(num_additional_samples_repos));
    a->Visit(DRAKE_NVP(num_additional_samples_c3));
    a->Visit(DRAKE_NVP(spline_width));
    a->Visit(DRAKE_NVP(switching_hysteresis));
    a->Visit(DRAKE_NVP(reposition_fixed_cost));
    a->Visit(DRAKE_NVP(finished_reposition_cost));
    a->Visit(DRAKE_NVP(travel_cost_per_meter));
    a->Visit(DRAKE_NVP(travel_speed));
    a->Visit(DRAKE_NVP(num_threads));
    
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
  bool use_full_cost;
  double Q_default;
  double Q_finger;
  double Q_ball_x;
  double Q_ball_y;
  double Q_ball_z;
  double Q_finger_vel;
  double Q_ball_vel;
  double R;
  double G;
  double G_ground;
  double U_default;
  double U_pos;
  double U_vel;
  double U_u;
  VectorXd q_init_finger;
  VectorXd q_init_ball_c3;
  double dt;
  double velocity_limit;
  double orientation_degrees;
  int axis_option;
  double planning_timestep;
  double c3_planned_next_state_timestep;
  int horizon_length;

  // initialization/simulation parameters
  VectorXd q_init_franka;
  VectorXd q_init_ball;
  VectorXd initial_start;
  VectorXd initial_finish;
  double stabilize_time1;
  double move_time;
  double stabilize_time2;
  double sim_dt;
  double sim_publish_dt;
  double realtime_rate;

  // trajectory parameters
  double traj_radius;
  double phase;
  double x_c;
  double y_c;
  double degree_increment;
  double time_increment;
  int hold_order;
  double fixed_goal_x;
  double fixed_goal_y;

  // geometry parameters
  double jack_half_width;
  double finger_radius;
  VectorXd EE_offset;
  double table_offset;
  double model_table_offset;

  //sampling parameters
  bool force_skip_sampling;
  int sampling_strategy;
  double sampling_radius;
  double sample_height;
  double min_angle_from_vertical;
  double max_angle_from_vertical;
  int num_additional_samples_repos;
  int num_additional_samples_c3;
  double spline_width;

  //switching parameters
  double switching_hysteresis;
  double reposition_fixed_cost;
  double finished_reposition_cost;
  double travel_cost_per_meter;
  double travel_speed;
  int num_threads;

  // misc
  double contact_threshold;
  int enable_heuristic;
  int enable_contact;
  double ball_stddev;
  
  //Visualization
  bool display_rollout_flag;

  // test parameters
  double lead_angle;
  int trajectory_type;

  // filter parameters
  uint32_t dt_filter_length;
  double alpha_p;
  double alpha_v;

};