#pragma once

#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"

using std::map;
using std::vector;
using std::is_same;
using std::string;

using Eigen::Map;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;

using drake::AutoDiffXd;
using drake::VectorX;
using drake::MatrixX;
using drake::math::AutoDiffMatrixType;
using drake::math::initializeAutoDiff;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::systems::RigidBodyPlant;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;


namespace dairlib {


int GetBodyIndexFromNamePlanar(const RigidBodyTree<double>& tree, 
                         string name);

VectorXd ComputePlanarJointLimitForces(RigidBodyPlant<double>* plant, 
                                       VectorXd x_init);

bool PlanarJointsWithinLimits(const RigidBodyTree<double>& tree, 
                              VectorXd x,
                              double tolerance = 0.0,
                              bool print_debug_messages = true);


struct ContactInfo2 {
    VectorXd phi;
    Matrix3Xd normal;
    Matrix3Xd xA;
    Matrix3Xd xB;
    vector<int> idxA;
    vector<int> idxB;
    vector<Map<Matrix3Xd>> tangents_map_vector;
};

template<typename T>
class PlanarPlant {

  public:
    PlanarPlant(const RigidBodyPlant<T>& plant):plant_(plant),
                                          tree_(plant_.get_rigid_body_tree()) {}


    MatrixX<T> CalcContactJacobianPlanar(VectorX<T> q,
                                         VectorX<T> v,
                                         int num_contact_constraints) const;
    
    MatrixX<T> CalcContactJacobianPlanar(VectorX<T> q, 
                                         VectorX<T> v,
                                         int num_contact_constraints,
                                         ContactInfo2 contact_info) const;
                                         


    void CalcTimeDerivativesPlanar(VectorX<T> x,
                                   VectorX<T> u, 
                                   ContinuousState<T>* x_dot) const;

    VectorX<T> CalcTimeDerivativesPlanar(VectorX<T> x,
                                         VectorX<T> u) const;

    void CalcTimeDerivativesPlanar(VectorX<T> x, 
                                   VectorX<T> u, 
                                   VectorX<T> lambda, 
                                   ContinuousState<T>* x_dot) const;

    VectorX<T> CalcTimeDerivativesPlanar(VectorX<T> x,
                                         VectorX<T> u, 
                                         VectorX<T> lambda) const;

    void CalcTimeDerivativesPlanarStanding(VectorX<T> x, 
                                           VectorX<T> u, 
                                           VectorX<T> lambda,
                                           ContinuousState<T>* x_dot) const;

    VectorX<T> CalcTimeDerivativesPlanarStanding(VectorX<T> x, 
                                                 VectorX<T> u, 
                                                 VectorX<T> lambda) const;

    VectorX<T> CalcMVdotPlanarStanding(VectorX<T> x, 
                                       VectorX<T> u,
                                       VectorX<T> lambda) const;

    const RigidBodyPlant<T>& plant_;
    const RigidBodyTree<double>& tree_;

};


}  // namespace dairlib
