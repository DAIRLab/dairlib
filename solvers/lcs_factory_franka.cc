#include "solvers/lcs_factory_franka.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/solvers/moby_lcp_solver.h"

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

std::pair<LCS,double> LCSFactoryFranka::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, double mu, float dt) {


  ///
  /// First, calculate vdot and derivatives from non-contact dynamcs
  ///

  AutoDiffVecXd C(plant.num_velocities());

  plant_ad.CalcBiasTerm(context_ad, &C);

  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() *
                     plant_ad.get_actuation_input_port().Eval(context_ad);

  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);


  MatrixX<AutoDiffXd> M(plant.num_velocities(), plant.num_velocities());
  plant_ad.CalcMassMatrix(context_ad, &M);

  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + f_app.generalized_forces() + Bu - C);

  // Constant term in dynamics, d_vv = d + A x_0 + B u_0
  VectorXd d_vv = ExtractValue(vdot_no_contact);

  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);

  VectorXd inp_dvv = plant.get_actuation_input_port().Eval(context);
  VectorXd x_dvv(plant.num_positions() + plant.num_velocities() + plant.num_actuators());
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), inp_dvv;
  VectorXd x_dvvcomp = AB_v * x_dvv;

  VectorXd d_v = d_vv - x_dvvcomp;


  int n_state = plant_ad.num_positions();
  int n_vel = plant_ad.num_velocities();
  int n_total = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_input = plant_ad.num_actuators();


  ///////////
  AutoDiffVecXd qdot_no_contact(plant.num_positions());


  AutoDiffVecXd state = plant_ad.get_state_output_port().Eval(context_ad);

  AutoDiffVecXd vel = state.tail(n_vel);

  plant_ad.MapVelocityToQDot(context_ad, vel, &qdot_no_contact);

  MatrixXd AB_q = ExtractGradient(qdot_no_contact);

  MatrixXd d_q = ExtractValue(qdot_no_contact);


  MatrixXd Nq = AB_q.block(0, n_state, n_state, n_vel);


  ///
  /// Contact-related terms
  ///
  VectorXd phi(contact_geoms.size());
  MatrixXd J_n(contact_geoms.size(), plant.num_velocities());
  MatrixXd J_t(2 * contact_geoms.size() * num_friction_directions,
               plant.num_velocities());

  for (int i = 0; i < contact_geoms.size(); i++) {
    multibody::GeomGeomCollider collider(
        plant, contact_geoms[i]);  // deleted num_fricton_directions (check with
                                   // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);

//    std::cout << "phi_i" << std::endl;
//    std::cout << phi_i << std::endl;
//    std::cout << "phi_i" << std::endl;


    phi(i) = phi_i;

    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
              plant.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_directions, plant.num_velocities());
  }

//  std::cout << "here" << std::endl;
//  std::cout << "phi\n" << phi << std::endl;

  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  /// std::cout << MinvJ_t_T.cols() << std::endl;

  //float dt = 0.1;
  auto n_contact = 2 * contact_geoms.size() +
                   2 * contact_geoms.size() * num_friction_directions;

  // std::cout << "nc:" << n_contact  << std::endl;

  MatrixXd A(n_total, n_total);
  MatrixXd B(n_total, n_input);
  MatrixXd D(n_total, n_contact);
  VectorXd d(n_total);
  MatrixXd E(n_contact, n_total);
  MatrixXd F(n_contact, n_contact);
  MatrixXd H(n_contact, n_input);
  VectorXd c(n_contact);

  MatrixXd AB_v_q = AB_v.block(0, 0, n_vel, n_state);
  MatrixXd AB_v_v = AB_v.block(0, n_state, n_vel, n_vel);
  MatrixXd AB_v_u = AB_v.block(0, n_total, n_vel, n_input);

  A.block(0, 0, n_state, n_state) =
      MatrixXd::Identity(n_state, n_state) + dt * dt * Nq * AB_v_q;
  A.block(0, n_state, n_state, n_vel) = dt * Nq + dt * dt * Nq * AB_v_v;
  A.block(n_state, 0, n_vel, n_state) = dt * AB_v_q;
  A.block(n_state, n_state, n_vel, n_vel) =
      dt * AB_v_v + MatrixXd::Identity(n_vel, n_vel);

  B.block(0, 0, n_state, n_input) = dt * dt * Nq * AB_v_u;
  B.block(n_state, 0, n_vel, n_input) = dt * AB_v_u;

  D = MatrixXd::Zero(n_total, n_contact);
  D.block(0, 2 * contact_geoms.size(), n_state,
          2 * contact_geoms.size() * num_friction_directions) =
      dt * dt * Nq * MinvJ_t_T;
  D.block(n_state, 2 * contact_geoms.size(), n_vel,
          2 * contact_geoms.size() * num_friction_directions) = dt * MinvJ_t_T;

  D.block(0, contact_geoms.size(), n_state, contact_geoms.size() )  =  dt * dt * Nq * MinvJ_n_T;

  D.block(n_state, contact_geoms.size(), n_vel, contact_geoms.size() ) = dt * MinvJ_n_T;

  d.head(n_state) = dt * dt * Nq * d_v;
  d.tail(n_vel) = dt * d_v;

  E = MatrixXd::Zero(n_contact, n_total);
  E.block(contact_geoms.size(), 0, contact_geoms.size(), n_state) =
      dt * dt * J_n * AB_v_q;
  E.block(2 * contact_geoms.size(), 0,
          2 * contact_geoms.size() * num_friction_directions, n_state) =
      dt * J_t * AB_v_q;
  E.block(contact_geoms.size(), n_state, contact_geoms.size(), n_vel) =
      dt * J_n + dt * dt * J_n * AB_v_v;
  E.block(2 * contact_geoms.size(), n_state,
          2 * contact_geoms.size() * num_friction_directions, n_vel) =
      J_t + dt * J_t * AB_v_v;

  MatrixXd E_t = MatrixXd::Zero(
      contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions);
  for (int i = 0; i < contact_geoms.size(); i++) {
    E_t.block(i, i * (2 * num_friction_directions), 1,
              2 * num_friction_directions) =
        MatrixXd::Ones(1, 2 * num_friction_directions);
  };

  F = MatrixXd::Zero(n_contact, n_contact);
  F.block(0, contact_geoms.size(), contact_geoms.size(), contact_geoms.size()) =
      mu * MatrixXd::Identity(contact_geoms.size(), contact_geoms.size());
  F.block(0, 2 * contact_geoms.size(), contact_geoms.size(),
          2 * contact_geoms.size() * num_friction_directions) = -E_t;

  F.block(contact_geoms.size(), contact_geoms.size(), contact_geoms.size(), contact_geoms.size() ) = dt * dt * J_n * MinvJ_n_T;

  F.block(contact_geoms.size(), 2 * contact_geoms.size(), contact_geoms.size(),
          2 * contact_geoms.size() * num_friction_directions) =
      dt * dt * J_n * MinvJ_t_T;

  F.block(2 * contact_geoms.size(), 0,
          2 * contact_geoms.size() * num_friction_directions,
          contact_geoms.size()) = E_t.transpose();

  F.block(2 * contact_geoms.size(), contact_geoms.size(), contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions ) = dt * J_t * MinvJ_n_T;

  F.block(2 * contact_geoms.size(), 2 * contact_geoms.size(),
          2 * contact_geoms.size() * num_friction_directions,
          2 * contact_geoms.size() * num_friction_directions) =
      dt * J_t * MinvJ_t_T;

  H = MatrixXd::Zero(n_contact, n_input);
  H.block(contact_geoms.size(), 0, contact_geoms.size(), n_input) =
      dt * dt * J_n * AB_v_u;
  H.block(2 * contact_geoms.size(), 0,
          2 * contact_geoms.size() * num_friction_directions, n_input) =
      dt * J_t * AB_v_u;


  c = VectorXd::Zero(n_contact);
  c.segment(contact_geoms.size(), contact_geoms.size()) =
      phi + dt * dt * J_n * d_v;
  c.segment(2 * contact_geoms.size(),
            2 * contact_geoms.size() * num_friction_directions) =
      J_t * dt * d_v;


  int N = 5;

  auto Dn = D.squaredNorm();
  auto An = A.squaredNorm();
  auto AnDn = An / Dn;

  std::vector<MatrixXd> A_lcs(N, A);
  std::vector<MatrixXd> B_lcs(N, B);
  std::vector<MatrixXd> D_lcs(N, D * AnDn);
  std::vector<VectorXd> d_lcs(N, d );
  std::vector<MatrixXd> E_lcs(N, E / AnDn);
  std::vector<MatrixXd> F_lcs(N, F);
  std::vector<VectorXd> c_lcs(N, c / AnDn);
  std::vector<MatrixXd> H_lcs(N, H / AnDn);

  LCS system(A_lcs, B_lcs, D_lcs, d_lcs, E_lcs, F_lcs, H_lcs, c_lcs);

//    std::cout << "dstahp" << std::endl;
//  std::cout << c_lcs[0] << std::endl;
//  std::cout << "dstahp" << std::endl;

////////
//  ///check LCS predictions
//  VectorXd inp = plant.get_actuation_input_port().Eval(context);
//
//  //std::cout << inp << std::endl;
//
//  VectorXd x0(plant.num_positions() + plant.num_velocities());
//  x0 << plant.GetPositions(context), plant.GetVelocities(context);
////
////  std::cout << "real" << std::endl;
//// std::cout << plant_ad.GetVelocities(context_ad) << std::endl;
////
//  VectorXd asd = system.Simulate(x0 ,inp);
//
//  // calculate force
//  drake::solvers::MobyLCPSolver<double> LCPSolver;
//  VectorXd force;
//
//  VectorXd x_init = x0;
//  VectorXd input = inp;
//
//
//  auto flag = LCPSolver.SolveLcpLemke(F, E * x_init + c + H * input,
//                                      &force);
//
//  //VectorXd x_final = A * x_init + B * input + D * force + d;
//
//  //if (flag == 1){
//
//      std::cout << "LCS force estimate" << std::endl;
//    std::cout << force << std::endl;
//    std::cout << "LCS force estimate" << std::endl;
////
////
////        std::cout << "Jn * v" << std::endl;
////   std::cout << J_n * x_final.tail(9) << std::endl;
////    std::cout << "Jn * v" << std::endl;
//
////        std::cout << "gap" << std::endl;
////   std::cout << E * x_init + c + H * input + F * force << std::endl;
////    std::cout << "gap" << std::endl;
//
//    std::cout << "phi" << std::endl;
//    std::cout << phi << std::endl;
//    std::cout << "phi" << std::endl;
//
//
//  }

//
// std::cout << "prediction" << std::endl;
// std::cout << asd.tail(15) << std::endl;

  std::pair <LCS, double> ret (system, AnDn);

  return ret;

}

}  // namespace solvers
}  // namespace dairlib
