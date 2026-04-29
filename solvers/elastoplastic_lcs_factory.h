#pragma once

#include <set>

#include "multibody/geom_geom_collider.h"
#include "solvers/lcs.h"
#include "solvers/lcs_factory.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using std::vector;

namespace dairlib {
namespace solvers {

class PlasticNetworkLCSFactory {
 public:
  /// Build a time-invariant LCS that represents a system including the dynamics
  /// of an elastoplastic network linearized about a given state.  The
  /// complementarity variables include internal plasticity forces, in addition
  /// to any external contacts.
  /// NOTE:  Is compatible with LCSFactory::PreProcessor to obtain
  /// external_contact_geoms that can then be passed into this method.
  static LCS ToLCS(
      const MultibodyPlant<double>& plant, const Context<double>& context,
      const MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      const Context<drake::AutoDiffXd>& context_ad,
      const vector<SortedPair<GeometryId>>& external_contact_geoms,
      const vector<SortedPair<GeometryId>>& internal_contact_geoms,
      const Eigen::VectorXd& yield_forces, const vector<double>& mu,
      const double& dt, const int& N, int n_lambda_with_tangential,
      const vector<int>& num_friction_directions_per_contact,
      const vector<int>& starting_index_per_contact_in_lambda_t_vector,
      ContactModel contact_model);

 private:
  /// Nothing for now.
};

}  // namespace solvers
}  // namespace dairlib
