#pragma once

namespace dairlib {

/// Outputs a nominal stand state into the xState vector pointer based on the 
/// height. This is an approximation for use in initial conditions.
///   @param plant a pointer to the MultibodyPlant
///   @param xState a pointer to a vector for the state to be saved in
///   @param height the nominal height for calculating approx inv kinematics

template <typename T>
void nominalSpiritStand(
    drake::multibody::MultibodyPlant<T>& plant, 
    Eigen::VectorXd& xState, 
    double height);

/// Returns a pointer to the toe frame by index. On Spirit the toes are as follows
/// Front Left (0), Back Left (1), Front Right (2), Back Right(3)
/// @param plant a point to the MultibodyPlant 
/// @param toeIndex The index of the desired toe (0-3)
template <typename T>
const drake::multibody::Frame<T>& getSpiritToeFrame( 
    drake::multibody::MultibodyPlant<T>& plant, 
    u_int8_t toeIndex );

/// Returns a unique WorldPointEvaluator for the toe at toeIndex. Uses the normal
/// vector constructor to allow for non-world-aligned ground/surface contact.
/// Also includes friction, and a offset from the frame if the toes have a radius
///   @param plant a point to the MultibodyPlant
///   @param toeIndex the toeIndex of the desired evaluator
///   @param toePoint the toeOffset in the toe frame
///   @param mu frictional coefficient
///   @param normal the contact normal of this evaluator
///   @param xy_active are the tangent directions active
///              )
///
template <typename T>
std::unique_ptr<multibody::WorldPointEvaluator<T>> getSpiritToeEvaluator( 
                      drake::multibody::MultibodyPlant<T>& plant, 
                      u_int8_t toeIndex,
                      const Eigen::Vector3d toePoint = Eigen::Vector3d::Zero(),
                      double mu = 0.0,
                      const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(),
                      bool xy_active = true
                      );


/// Takes in a matrix of bools that define the mode sequence (contact/no contact)
/// and outputs a tuple of the mode vector, toe evaluator sets, and toe evaluators
/// the latter two are only used to avoid descoping of the references used by
/// the mode sequence
///    @param plant a pointer to a multibodyPlant
///    @param modeSeqMat a bool matrix describing toe contacts as true or false 
///             e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
///    @param knotpointMat  Vector of knot points for each mode  
///    @param mu Friction coefficient

template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>   
          >     
    createSpiritModeSequence( 
          drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
          Eigen::Matrix<bool,-1,4> modeSeqMat, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
          Eigen::VectorXi knotpointMat, // Matrix of knot points for each mode  
          double mu = 1);

}

