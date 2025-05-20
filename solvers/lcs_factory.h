#pragma once

#include <set>

#include "solvers/lcs.h"
#include "multibody/geom_geom_collider.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/geometry_ids.h"   
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/math/rigid_transform.h" 

using std::vector;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::math::RigidTransform;
using drake::geometry::FrameId;
using drake::systems::Context;
using drake::geometry::SignedDistancePair;
namespace dairlib {
namespace solvers {

enum class ContactModel {
  kStewartAndTrinkle,  /// Stewart and Trinkle timestepping contact
  kAnitescu            /// Anitescu convex contact
};

class LCSFactory {
 public:
  /// Build a time-invariant LCS, linearizing a MultibodyPlant
  /// Contacts are specified by the pairs in contact_geoms. Each element
  /// in the contact_geoms vector defines a collision.
  /// This method also uses two copies of the Context, one for double and one
  /// for AutoDiff. Given that Contexts can be expensive to create, this is
  /// preferred to extracting the double-version from the AutoDiff.
  ///
  /// TODO: add variant allowing for different frictional properties per
  ///       contact
  ///
  /// @param plant The standard <double> MultibodyPlant
  /// @param context The context about which to linearize (double)
  /// @param plant_ad An AutoDiffXd templated plant for gradient calculation
  /// @param context The context about which to linearize (AutoDiffXd)
  /// @param contact_geoms
  /// @param num_friction faces
  /// @param mu
  /// @oaram dt The timestep for discretization
  /// @param N
  static LCS LinearizePlantToLCS(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::systems::Context<double>& context,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      const drake::systems::Context<drake::AutoDiffXd>& context_ad,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
          contact_geoms,
      int num_friction_directions, const std::vector<double>& mu, double dt,
      int N, ContactModel = ContactModel::kStewartAndTrinkle);

  static std::pair<Eigen::MatrixXd, std::vector<Eigen::VectorXd>> ComputeContactJacobian(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::systems::Context<double>& context,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
          contact_geoms,
      int num_friction_directions, const std::vector<double>& mu,
      ContactModel = ContactModel::kStewartAndTrinkle);

  /// Create an LCS by fixing some modes from another LCS
  /// Ignores generated inequalities that correspond to these modes, but
  /// these could be returned via pointer if useful
  ///
  /// @param active_lambda_inds The indices for lambda thta must be non-zero
  /// @param inactive_lambda_inds The indices for lambda that must be 0
  static LCS FixSomeModes(const LCS& other, std::set<int> active_lambda_inds,
                          std::set<int> inactive_lambda_inds);

    /// Preprocess the contact pairs to select the closest contacts
    /// @param plant The MultibodyPlant
    /// @param context The context about which to linearize
    /// @param contact_geoms The contact geometries
    /// @param resolve_contacts_to_list The number of contacts to resolve
    /// @param num_friction_directions The number of friction directions
    /// @param num_contacts The number of contacts
    /// @param verbose Whether to print verbose information
    /// @return The closest contacts
    static std::vector<drake::SortedPair<drake::geometry::GeometryId>> PreProcessor(
            const drake::multibody::MultibodyPlant<double>& plant,
            const drake::systems::Context<double>& context,
            const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
                contact_geoms,
            const std::vector<int>& resolve_contacts_to_list,
            int num_friction_directions, int num_contacts, bool verbose = false);
    /// Print verbose contact information
    /// @param plant The MultibodyPlant
    /// @param context The context about which to linearize
    /// @param pair The contact pair
    /// @param phi_i The sign distance
    static void PrintVerboseContactInfo(
        const drake::multibody::MultibodyPlant<double>& plant,
        const drake::systems::Context<double>& context,
        const drake::SortedPair<drake::geometry::GeometryId>& pair,
        const double phi_i);
};

}  // namespace solvers
}  // namespace dairlib
