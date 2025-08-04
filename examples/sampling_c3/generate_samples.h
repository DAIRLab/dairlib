#include <random>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <numbers>

#include "common/math_utils.h"
#include "common/update_context.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/sampling_params.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_options.h"
#include <drake/geometry/query_object.h>
#include "systems/controllers/face.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using dairlib::systems::Face;

namespace dairlib {
namespace systems {

/// Public function
std::vector<Eigen::VectorXd> GenerateSampleStates(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    const bool& is_doing_c3,
    const SamplingParams& sampling_params,
    const SamplingC3Options& sampling_c3_options,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    std::vector<Face> faces,
    std::vector<double> face_bins,
    std::vector<std::vector<Face>> faces_per_object,
    std::vector<std::vector<double>> face_bins_per_object,
    std::vector<double> total_area_per_object,
    bool include_walls,
    std::vector<bool> object_on_target
);

/// Individual sampling strategies returning 3D EE position
Eigen::Vector3d RadiallySymmetricSampling(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const int& num_samples,
    const int& i,
    const double& sampling_radius,
    const double& sampling_height
);

Eigen::Vector3d RandomOnCircleSampling(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& sampling_height
);

Eigen::Vector3d RandomOnSphereSampling(
    const int& n_q,
    const int& n_v,
    const Eigen::VectorXd& x_lcs,
    const double& sampling_radius,
    const double& min_angle_from_vertical,
    const double& max_angle_from_vertical
);

Eigen::Vector3d FixedSample(
    const Eigen::Vector3d& fixed_sample_location
);

Eigen::Vector3d PerimeterSampling(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options
);

Eigen::Vector3d ShellSampling(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    const SamplingParams& sampling_params,
    const SamplingC3Options sampling_c3_options
);

Eigen::VectorXd MeshNormalSampling(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
    drake::systems::Context<double>* context, 
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const SamplingParams& sampling_params,
    const drake::geometry::QueryObject<double>& query_object,
    std::vector<Face> faces,
    std::vector<double> face_bins);

Eigen::VectorXd MeshNormalSamplingMultiObject(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const Eigen::VectorXd& x_lcs,
    drake::multibody::MultibodyPlant<double>& plant, 
    drake::systems::Context<double>* context, 
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<std::vector<drake::SortedPair<drake::geometry::GeometryId>>>& contact_geoms,
    const SamplingParams& sampling_params,
    const drake::geometry::QueryObject<double>& query_object,
    std::vector<std::vector<Face>> faces_per_object,
    std::vector<std::vector<double>> face_bins_per_object,
    std::vector<double> total_area_per_object,
    bool include_walls,
    std::vector<bool> object_on_target
);


/// Helper functions
int FindBin(const double* bins, int n, double x);

/// Whether the candidate state's EE location is within the robot workspace.
bool IsSampleInWorkspace(
    const Eigen::VectorXd& candidate_state,
    const SamplingC3Options& sampling_c3_options
);

/// Find and return the end effector radius from the plant.
/// WARNING:  assumes EE-object pairs are first in contact_geoms.
double GetEERadiusFromPlant(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms
);

/// Whether the candidate state's EE location is within a clearance distance of
/// the surface of the object.  Can factor in the EE radius if a real clearance
/// distance between the EE and the object is desired; otherwise this is the
/// distance from the center of the EE to the surface of the object.
bool IsSampleWithinDistanceOfSurface(
    const int& n_q,
    const int& n_v,
    const int& n_u,
    const double& clearance_distance,
    const Eigen::VectorXd& candidate_state,
    drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms,
    SamplingC3Options sampling_c3_options,
    int& min_distance_index
);

/// Project an EE candidate location outside the object such that the clearance
/// distance in the sampling parameters is satisfied.
Eigen::VectorXd ProjectSampleOutsideObject(
    Eigen::VectorXd& candidate_state,
    int min_distance_index,
    const SamplingParams& sampling_params,
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    const std::vector<
        std::vector<drake::SortedPair<drake::geometry::GeometryId>>>&
        contact_geoms
);

}  // namespace systems
}  // namespace dairlib