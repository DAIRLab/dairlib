#include "solvers/lcs_factory.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::VectorXd;

LCS LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, double mu) {

  ///
  /// First, calculate vdot and derivatives from non-contact dynamcs
  ///



  AutoDiffVecXd C(plant.num_velocities());

  //0std::cout << ExtractGradient(C).cols() << std::endl;

  plant_ad.CalcBiasTerm(context_ad, &C);

  //std::cout << ExtractGradient(C).cols() << std::endl;

  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() *
                     plant_ad.get_actuation_input_port().Eval(context_ad);

  /*
  auto u = plant.get_actuation_input_port().Eval(context);
  auto u_ad = drake::math::InitializeAutoDiff(u);
  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() * u_ad;
  */

  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(plant.num_velocities(), plant.num_velocities());
  plant_ad.CalcMassMatrix(context_ad, &M);

  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + f_app.generalized_forces() + Bu - C);

  // Constant term in dynamics, d
  VectorXd d_v = ExtractValue(vdot_no_contact);

  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);

  int n_state = plant_ad.num_positions();
  int n_vel = plant_ad.num_velocities();
  int n_total = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_input = plant_ad.num_actuators();

  //std::cout << n_state << std::endl;
  //std::cout << n_vel << std::endl;

  ///////////
  AutoDiffVecXd qdot_no_contact(plant.num_positions());
  AutoDiffVecXd state = plant_ad.get_state_output_port().Eval(context_ad);
  AutoDiffVecXd vel = state.tail(n_vel);
  //std::cout << std::size(state) << std::endl;
  plant_ad.MapVelocityToQDot(context_ad, vel, &qdot_no_contact);
  MatrixXd AB_q = ExtractGradient(qdot_no_contact);
  MatrixXd d_q = ExtractValue(qdot_no_contact);

  //std::cout << AB_q.rows() << "x" << AB_q.cols() << std::endl;

  //std::cout << AB_q.block(0,n_state, n_state, n_vel) << std::endl;

  MatrixXd Nq = AB_q.block(0,n_state, n_state, n_vel);

  ///////////


  ///
  /// Contact-related terms
  ///
  VectorXd phi(contact_geoms.size());
  MatrixXd J_n(contact_geoms.size(), plant.num_velocities());
  MatrixXd J_t(2 * contact_geoms.size() * num_friction_directions,
               plant.num_velocities());
  for (int i = 0; i < contact_geoms.size(); i++) {
    multibody::GeomGeomCollider collider(plant, contact_geoms[i],
                                         num_friction_directions);
    auto [phi_i, J_i] = collider.Eval(context);
    phi(i) = phi_i;
    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
              plant.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_directions, plant.num_velocities());
  }

  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  std::cout << MinvJ_t_T.cols() << std::endl;

  float dt = 0.1;
  auto n_contact = contact_geoms.size() + 2 * contact_geoms.size() * num_friction_directions;

  MatrixXd A(n_total,n_total);
  MatrixXd B(n_total, n_input);
  MatrixXd D(n_total, n_contact);
  VectorXd d(n_total);
  MatrixXd E(n_contact, n_total);
  MatrixXd F(n_contact, n_contact);
  MatrixXd H(n_contact, n_input);
  VectorXd c(n_contact);




  MatrixXd hold = dt * AB_q;
  MatrixXd hold2 = dt * AB_v;
  A.block(0,0,n_state,n_state) = MatrixXd::Identity(n_state,n_state) + hold.block(0,0,n_state,n_state);
  A.block(0,n_state,n_state,n_vel) = hold.block(0,n_state,n_state,n_vel);
  A.block(n_state,0,n_vel,n_state) = hold2.block(0,0,n_vel,n_state);
  A.block(n_state,n_state, n_vel, n_vel) = hold2.block(0,n_state, n_vel, n_vel) + MatrixXd::Identity(n_vel,n_vel);

  B.block(0,0,n_state,n_input) = dt * AB_q.block(0,n_total,n_state,n_input);
  B.block(n_state,0,n_vel,n_input) = dt * AB_v.block(0,n_total,n_state,n_input);

  MatrixXd hold3 = dt * dt * Nq * MinvJ_t_T;
  D.block(0,0,n_state,n_contact) = hold3;
  D.block(n_state,0,n_vel,n_contact) = MinvJ_t_T * dt;

  d.head(n_state) = d_q;
  d.tail(n_vel) = d_v;




  //std::cout << MinvJ_t_T.rows() << std::endl;

  //std::cout << (Nq * AB_v).rows() << std::endl;


  //A.block(0,n_state,)

  /// TODO: finish by assembling the terms computed above into the LCS structure
  ///       and returning an LCS object
  ///  v_{k+1} = v_k + dt * (AB_V * [x; u] + Minv*J_T*lambda)
  ///  q_{k+1} = q_k + dt * Nq * v_{k+1}
  ///  phi_{k+1} = phi + J_n * dt * v_{k+1}
  ///  tangential velocity: J_t * v_{k+1}

  int N = 10;

  LCS system(A, B, D, d, E, F, H, c, N);

  return system;

}

}  // namespace solvers
}  // namespace dairlib
