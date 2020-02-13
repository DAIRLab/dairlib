#include "examples/goldilocks_models/find_models/traj_opt_helper_func.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"

#include "drake/lcm/drake_lcm.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"

#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/choose_best_solver.h"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include "drake/math/rotation_matrix.h"

#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "common/find_resource.h"


using drake::VectorX;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MatrixXDecisionVariable;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using drake::trajectories::PiecewisePolynomial;

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

using drake::trajectories::PiecewisePolynomial;

using drake::math::RotationMatrix;
using drake::math::RollPitchYaw;

namespace dairlib {
namespace goldilocks_models {

// Do inverse kinematics to get configuration guess
vector<VectorXd> GetCassieInitGuessForQ(int N,
                                        double stride_length,
                                        const MultibodyPlant<double>& plant) {
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);

  vector<VectorXd> q_init_guess;
  VectorXd q_ik_guess = VectorXd::Zero(n_q);
  Eigen::Vector4d quat(2000.06,
                       -0.339462,
                       -0.609533,
                       -0.760854);
  q_ik_guess << quat.normalized(),
             0.000889849,
             0.000626865,
             1.0009,
             -0.0112109,
             0.00927845,
             -0.000600725,
             -0.000895805,
             1.15086,
             0.610808,
             -1.38608,
             -1.35926,
             0.806192,
             1.00716,
             -M_PI / 2,
             -M_PI / 2;

  for (int i = 0; i < N; i++) {
    double eps = 1e-3;
    Vector3d eps_vec = eps * VectorXd::Ones(3);
    Vector3d pelvis_pos(stride_length * i / (N - 1),
                        0.0,
                        1.0);
    Vector3d stance_toe_pos(stride_length / 2,
                            0.12,
                            0.05);
    Vector3d swing_toe_pos(-stride_length / 2 + 2 * stride_length * i / (N - 1),
                           -0.12,
                           0.05 + 0.1 * (-abs((i - N / 2.0) / (N / 2.0)) + 1));
    // cout << "swing foot height = " <<
    //      0.05 + 0.1 * (-abs((i - N / 2.0) / (N / 2.0)) + 1);

    const auto & world_frame = plant.world_frame();
    const auto & pelvis_frame = plant.GetFrameByName("pelvis");
    const auto & toe_left_frame = plant.GetFrameByName("toe_left");
    const auto & toe_right_frame = plant.GetFrameByName("toe_right");

    drake::multibody::InverseKinematics ik(plant);
    ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0),
                             world_frame,
                             pelvis_pos - eps * VectorXd::Ones(3),
                             pelvis_pos + eps * VectorXd::Ones(3));
    ik.AddOrientationConstraint(pelvis_frame, RotationMatrix<double>(),
                                world_frame, RotationMatrix<double>(),
                                eps);
    ik.AddPositionConstraint(toe_left_frame, Vector3d(0, 0, 0),
                             world_frame,
                             stance_toe_pos - eps_vec,
                             stance_toe_pos + eps_vec);
    ik.AddPositionConstraint(toe_right_frame, Vector3d(0, 0, 0),
                             world_frame,
                             swing_toe_pos - eps_vec,
                             swing_toe_pos + eps_vec);
    ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_left")) == 0);
    ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_right")) == 0);
    // Four bar linkage constraint (without spring)
    ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_left"))
      + (ik.q())(positions_map.at("ankle_joint_left")) == M_PI * 13 / 180.0);
    ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_right"))
      + (ik.q())(positions_map.at("ankle_joint_right")) == M_PI * 13 / 180.0);

    ik.get_mutable_prog()->SetInitialGuess(ik.q(),
                                           q_ik_guess);
    const auto result = Solve(ik.prog());
    // SolutionResult solution_result = result.get_solution_result();
    // cout << "\n" << to_string(solution_result) << endl;
    // cout << "  Cost:" << result.get_optimal_cost() << std::endl;
    const auto q_sol = result.GetSolution(ik.q());
    // cout << "  q_sol = " << q_sol.transpose() << endl;
    VectorXd q_sol_normd(n_q);
    q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
    // cout << "  q_sol_normd = " << q_sol_normd << endl;
    q_ik_guess = q_sol_normd;
    q_init_guess.push_back(q_sol_normd);

    bool visualize_init_traj = false;
    if (visualize_init_traj) {
      // Build temporary diagram for visualization
      drake::systems::DiagramBuilder<double> builder_ik;
      SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<SceneGraph>();
      scene_graph_ik.set_name("scene_graph_ik");
      MultibodyPlant<double> plant_ik;
      multibody::addFlatTerrain(&plant_ik, &scene_graph_ik, .8, .8);
      Parser parser(&plant_ik, &scene_graph_ik);
      string full_name =
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
      parser.AddModelFromFile(full_name);
      plant_ik.mutable_gravity_field().set_gravity_vector(
        -9.81 * Eigen::Vector3d::UnitZ());
      plant_ik.Finalize();

      // Visualize
      VectorXd x_const = VectorXd::Zero(n_x);
      x_const.head(n_q) = q_sol;
      PiecewisePolynomial<double> pp_xtraj(x_const);

      multibody::connectTrajectoryVisualizer(&plant_ik, &builder_ik,
                                             &scene_graph_ik, pp_xtraj);
      auto diagram = builder_ik.Build();
      drake::systems::Simulator<double> simulator(*diagram);
      simulator.set_target_realtime_rate(.1);
      simulator.Initialize();
      simulator.AdvanceTo(1.0 / N);
    }
  }

  return q_init_guess;
}

// Get v by finite differencing q
vector<VectorXd> GetCassieInitGuessForV(const vector<VectorXd>& q_seed,
                                        double dt,
                                        const MultibodyPlant<double>& plant) {
  vector<VectorXd> qdot_seed;
  for (unsigned int i = 0; i < q_seed.size(); i++) {
    if (i == 0) {
      qdot_seed.push_back((q_seed[i + 1] - q_seed[i]) / dt);
    } else if ( i == q_seed.size() - 1) {
      qdot_seed.push_back((q_seed[i] - q_seed[i - 1]) / dt);
    } else {
      VectorXd v_plus = (q_seed[i + 1] - q_seed[i]) / dt;
      VectorXd v_minus = (q_seed[i] - q_seed[i - 1]) / dt;
      qdot_seed.push_back((v_plus + v_minus) / 2);
    }
  }

  // Convert qdot to v
  vector<VectorXd> v_seed;
  for (unsigned int i = 0; i < q_seed.size(); i++) {
    auto context = plant.CreateDefaultContext();
    plant.SetPositions(context.get(), q_seed[i]);
    VectorXd v(plant.num_velocities());
    plant.MapQDotToVelocity(*context, qdot_seed[i], &v);
    v_seed.push_back(v);
    // cout << i << ":\n";
    // cout << "  qdot = " << qdot_seed[i].transpose() << endl;
    // cout << "  v = " << v.transpose() << endl;
  }
  return v_seed;
}

}  // namespace goldilocks_models
}  // namespace dairlib
