#include "solvers/lcs_factory.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

using std::vector;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::VectorXd;
using Eigen::MatrixXd;

LCS LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant,
    const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
     const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_faces, double mu) {

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
  AutoDiffVecXd vdot_no_contact = M.ldlt().solve(
      tau_g + f_app.generalized_forces() + Bu - C);

  // Constant term in dynamics, d
  VectorXd d_v = ExtractValue(vdot_no_contact);

  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);

  ///
  /// Contact-related terms
  ///
  VectorXd phi(contact_geoms.size());
  MatrixXd J_n(contact_geoms.size(), plant.num_velocities());
  MatrixXd J_t(2 * contact_geoms.size() * num_friction_faces,
               plant.num_velocities());
  for (int i = 0; i < contact_geoms.size(); i++) {
    multibody::GeomGeomCollider collider(plant, contact_geoms[i],
                                         num_friction_faces);
    auto [phi_i, J_i] = collider.Eval(context);
    phi(i) = phi_i;
    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_faces, 0, 2 * num_friction_faces,
              plant.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_faces, plant.num_velocities());
  }


  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  /// TODO: finish by assembling the terms computed above into the LCS structure
  ///       and returning an LCS object
  ///  v_{k+1} = v_k + dt * (AB_V * [x; u] + Minv*J_T*lambda)
  ///  q_{k+1} = q_k + dt * v_{k+1}
  ///  phi_{k+1} = phi + J_n * dt * v_{k+1}
  ///  tangential velocity: J_t * v_{k+1}
}

} // namespace dairlib
} // namespace solvers


