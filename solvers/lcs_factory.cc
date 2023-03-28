#include "solvers/lcs_factory.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/solvers/moby_lcp_solver.h"

#include "drake/math/autodiff_gradient.h"



namespace dairlib {
namespace solvers {

using std::set;
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

std::pair<LCS,double> LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
int num_friction_directions, double mu, double dt, int N) {

///
/// First, calculate vdot and derivatives from non-contact dynamcs
///

int n_contacts = contact_geoms.size();

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
VectorXd phi(n_contacts);
MatrixXd J_n(n_contacts, plant.num_velocities());
MatrixXd J_t(2 * n_contacts * num_friction_directions,
             plant.num_velocities());

for (int i = 0; i < n_contacts; i++) {
multibody::GeomGeomCollider collider(
    plant, contact_geoms[i]);  // deleted num_fricton_directions (check with
// Michael about changes in geomgeom)
auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);

phi(i) = phi_i;

J_n.row(i) = J_i.row(0);
J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
plant.num_velocities()) =
J_i.block(1, 0, 2 * num_friction_directions, plant.num_velocities());
}

auto M_ldlt = ExtractValue(M).ldlt();
MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

//float dt = 0.1;
auto n_contact_vars = 2 * n_contacts +
    2 * n_contacts * num_friction_directions;

MatrixXd A(n_total, n_total);
MatrixXd B(n_total, n_input);
MatrixXd D(n_total, n_contact_vars);
VectorXd d(n_total);
MatrixXd E(n_contact_vars, n_total);
MatrixXd F(n_contact_vars, n_contact_vars);
MatrixXd H(n_contact_vars, n_input);
VectorXd c(n_contact_vars);

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

D = MatrixXd::Zero(n_total, n_contact_vars);
D.block(0, 2 * n_contacts, n_state,
2 * n_contacts * num_friction_directions) =
dt * dt * Nq * MinvJ_t_T;
D.block(n_state, 2 * n_contacts, n_vel,
2 * n_contacts * num_friction_directions) = dt * MinvJ_t_T;

D.block(0, n_contacts, n_state, n_contacts )  =  dt * dt * Nq * MinvJ_n_T;

D.block(n_state, n_contacts, n_vel, n_contacts ) = dt * MinvJ_n_T;

d.head(n_state) = dt * dt * Nq * d_v;
d.tail(n_vel) = dt * d_v;

E = MatrixXd::Zero(n_contact_vars, n_total);
E.block(n_contacts, 0, n_contacts, n_state) =
dt * dt * J_n * AB_v_q;
E.block(2 * n_contacts, 0,
2 * n_contacts * num_friction_directions, n_state) =
dt * J_t * AB_v_q;
E.block(n_contacts, n_state, n_contacts, n_vel) =
dt * J_n + dt * dt * J_n * AB_v_v;
E.block(2 * n_contacts, n_state,
2 * n_contacts * num_friction_directions, n_vel) =
J_t + dt * J_t * AB_v_v;

MatrixXd E_t = MatrixXd::Zero(
    n_contacts, 2 * n_contacts * num_friction_directions);
for (int i = 0; i < n_contacts; i++) {
E_t.block(i, i * (2 * num_friction_directions), 1,
2 * num_friction_directions) =
MatrixXd::Ones(1, 2 * num_friction_directions);
};


F = MatrixXd::Zero(n_contact_vars, n_contact_vars);
F.block(0, n_contacts, n_contacts, n_contacts) =
mu * MatrixXd::Identity(n_contacts, n_contacts);
F.block(0, 2 * n_contacts, n_contacts,
2 * n_contacts * num_friction_directions) = -E_t;

F.block(n_contacts, n_contacts, n_contacts, n_contacts ) =
dt * dt * J_n * MinvJ_n_T;
F.block(n_contacts, 2 * n_contacts, n_contacts,
2 * n_contacts * num_friction_directions) =
dt * dt * J_n * MinvJ_t_T;

F.block(2 * n_contacts, 0,
2 * n_contacts * num_friction_directions,
    n_contacts) = E_t.transpose();

F.block(2 * n_contacts, n_contacts, 2 * n_contacts * num_friction_directions,
    n_contacts) = dt * J_t * MinvJ_n_T;
F.block(2 * n_contacts, 2 * n_contacts,
2 * n_contacts * num_friction_directions,
2 * n_contacts * num_friction_directions) =
dt * J_t * MinvJ_t_T;
H = MatrixXd::Zero(n_contact_vars, n_input);
H.block(n_contacts, 0, n_contacts, n_input) =
dt * dt * J_n * AB_v_u;
H.block(2 * n_contacts, 0,
2 * n_contacts * num_friction_directions, n_input) =
dt * J_t * AB_v_u;

c = VectorXd::Zero(n_contact_vars);
c.segment(n_contacts, n_contacts) =
phi + dt * dt * J_n * d_v;

c.segment(2 * n_contacts,
2 * n_contacts * num_friction_directions) =
J_t * dt * d_v;

auto Dn = D.squaredNorm();
auto An = A.squaredNorm();
auto AnDn = An / Dn;

D *= AnDn;
E /= AnDn;
c /= AnDn;
H /= AnDn;

LCS system(A, B, D, d, E, F, H, c, N);


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


LCS LCSFactory::FixSomeModes(const LCS& other, set<int> active_lambda_inds,
                             set<int> inactive_lambda_inds) {

  vector<int> remaining_inds;

  // Assumes constant number of contacts per index
  int n_lambda = other.F_[0].rows();

  // Need to solve for lambda_active in terms of remaining elements
  // Build temporary [F1, F2] by eliminating rows for inactive
  for (int i = 0; i < n_lambda; i++) {
    // active/inactive must be exclusive
    DRAKE_ASSERT(!active_lambda_inds.count(i) ||
        !inactive_lambda_inds.count(i));

    // In C++20, could use contains instead of count
    if (!active_lambda_inds.count(i) &&
        !inactive_lambda_inds.count(i)) {
      remaining_inds.push_back(i);
    }
  }

  int n_remaining = remaining_inds.size();
  int n_active = active_lambda_inds.size();

  vector<MatrixXd> A, B, D, E, F, H;
  vector<VectorXd> d, c;

  // Build selection matrices:
  // S_a selects active indices
  // S_r selects remaining indices

  MatrixXd S_a = MatrixXd::Zero(n_active, n_lambda);
  MatrixXd S_r = MatrixXd::Zero(n_remaining, n_lambda);

  for (int i = 0; i < n_remaining; i++) {
    S_r(i, remaining_inds[i]) = 1;
  }
  {
    int i = 0;
    for (auto ind_j : active_lambda_inds) {
      S_a(i, ind_j) = 1;
      i++;
    }
  }


  for (int k = 0; k < other.N_; k++) {
    Eigen::BDCSVD<MatrixXd> svd;
    svd.setThreshold(1e-5);
    svd.compute(S_a * other.F_[k] * S_a.transpose(),
                Eigen::ComputeFullU | Eigen::ComputeFullV);

    // F_active likely to be low-rank due to friction, but that should be OK
    // MatrixXd res = svd.solve(F_ar);

    // Build new complementarity constraints
    // F_a_inv = pinv(S_a * F * S_a^T)
    // 0 <= \lambda_k \perp E_k x_k + F_k \lambda_k + H_k u_k + c_k
    // 0 = S_a *(E x + F S_a^T \lambda_a + F S_r^T \lambda_r + H_k u_k + c_k)
    // \lambda_a = -F_a_inv * (S_a F S_r^T * lambda_r + S_a E x + S_a H u + S_a c)
    //
    // 0 <= \lambda_r \perp S_r (I - F S_a^T F_a_inv S_a) E x + ...
    //                      S_r (I - F S_a^T F_a_inv S_a) F S_r^T \lambda_r + ...
    //                      S_r (I - F S_a^T F_a_inv S_a) H u + ...
    //                      S_r (I - F S_a^T F_a_inv S_a) c
    //
    // Calling L = S_r (I - F S_a^T F_a_inv S_a)S_r * other.D_[k]
    //  E_k = L E
    //  F_k = L F S_r^t
    //  H_k = L H
    //  c_k = L c
    // std::cout << S_r << std::endl << std::endl;
    // std::cout << other.F_[k] << std::endl << std::endl;
    // std::cout << other.F_[k] << std::endl << std::endl;
    // auto tmp = S_r * (MatrixXd::Identity(n_lambda, n_lambda) -
    //              other.F_[k] *  S_a.transpose());
    MatrixXd L = S_r * (MatrixXd::Identity(n_lambda, n_lambda) -
        other.F_[k] *  S_a.transpose() * svd.solve(S_a));
    MatrixXd E_k = L * other.E_[k];
    MatrixXd F_k = L * other.F_[k] * S_r.transpose();
    MatrixXd H_k = L * other.H_[k];
    MatrixXd c_k = L * other.c_[k];

    // Similarly,
    //  A_k = A - D * S_a^T * F_a_inv * S_a * E
    //  B_k = B - D * S_a^T * F_a_inv * S_a * H
    //  D_k = D * S_r^T - D * S_a^T  * F_a_inv * S_a F S_r^T
    //  d_k = d - D * S_a^T F_a_inv * S_a * c
    //
    //  Calling P = D * S_a^T * F_a_inv * S_a
    //
    //  A_k = A - P E
    //  B_k = B - P H
    //  D_k = S_r D - P S_r^T
    //  d_k = d - P c
    MatrixXd P = other.D_[k] * S_a.transpose() * svd.solve(S_a);
    MatrixXd A_k = other.A_[k] - P * other.E_[k];
    MatrixXd B_k = other.B_[k] - P * other.H_[k];
    MatrixXd D_k = other.D_[k] * S_r.transpose() - P * S_r.transpose();
    MatrixXd d_k = other.d_[k] - P * other.c_[k];
    E.push_back(E_k);
    F.push_back(F_k);
    H.push_back(H_k);
    c.push_back(c_k);
    A.push_back(A_k);
    B.push_back(B_k);
    D.push_back(D_k);
    d.push_back(d_k);
  }
  return LCS(A, B, D, d, E, F, H, c);
}

}  // namespace solvers
}  // namespace dairlib
