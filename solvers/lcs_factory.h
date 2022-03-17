#pragma once
#include "solvers/lcs.h"

namespace dairlib {
namespace solvers {

class LCSFactory {
 public:
  /// Build a time-invariant LCS, linearizing a MultibodyPlant
  /// Contacts are specified by the pairs in contact_geoms. Each elemnt
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
  static LCS LinearizePlantToLCS(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::systems::Context<double>& context,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      const drake::systems::Context<drake::AutoDiffXd>& context_ad,
      const std::vector<drake::SortedPair<drake::geometry::GeometryId>>& contact_geoms,
      int num_friction_directions,
      double mu);
};

}  // namespace solvers
}  // namespace dairlib
