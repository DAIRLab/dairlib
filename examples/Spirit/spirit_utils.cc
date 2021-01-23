#include <memory>
#include <chrono>
#include <tuple>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/spirit_utils.h"


using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace dairlib {

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;

/// Get a nominal Spirit Stand (i.e. zero hip ad/abduction motor torque, toes below motors) for initializing
template <typename T>
void nominalSpiritStand( MultibodyPlant<T>& plant, Eigen::VectorXd& xState, double height){
  //Get joint name dictionaries
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(plant);
  
  // Initialize state vector and add orienation and goal height
  xState = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  xState(positions_map.at("base_qw")) = 1;
  xState(positions_map.at("base_z")) = height;
  
  //Calculate the inverse kinematics to get the joint angles for toe placement
  double upperLegLength = 0.206; // length of the upper leg link
  double hipLength = 0.10098; //absOffset ToeLocation
  double hip2toeZLength = sqrt(height*height - hipLength*hipLength);// length of lower legs (2dof)
  double theta1 = asin(hip2toeZLength/(2*upperLegLength )) ; //upper angle
  double theta2 = 2*theta1 ; //lower angle
  double alpha = asin(hipLength/(height)); //hip angle

  int upperInd, kneeInd, hipInd;
  int mirroredFlag;
  for (int j = 0; j < 4; j++){
    upperInd = j * 2;
    kneeInd = j * 2 + 1;
    hipInd = j + 8;
    xState(positions_map.at("joint_" + std::to_string(upperInd))) = theta1 ;
    xState(positions_map.at("joint_" + std::to_string(kneeInd))) = theta2 ;
    mirroredFlag = -1;
    if ( hipInd > 9 ){
      mirroredFlag = 1;
    }
    xState(positions_map.at("joint_" + std::to_string(hipInd))) = mirroredFlag * alpha;
  }
}

template <typename T>
const drake::multibody::Frame<T>& getSpiritToeFrame( MultibodyPlant<T>& plant, u_int8_t toeIndex ){
  assert(toeIndex<4);
  return plant.GetFrameByName( "toe" + std::to_string(toeIndex) );
}

template <typename T>
std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>> getSpiritToeEvaluator( 
                      MultibodyPlant<T>& plant, 
                      u_int8_t toeIndex,
                      const Eigen::Vector3d toePoint ,
                      double mu ,
                      const Eigen::Vector3d normal ,
                      bool xy_active 
                      ){
  assert(toeIndex<4); // Check that toeIndex is an actual Spirit leg
  auto toe_eval =  std::make_unique<dairlib::multibody::WorldPointEvaluator<T>>(
        plant, 
        toePoint, 
        getSpiritToeFrame(plant, toeIndex ) , 
        normal, 
        Eigen::Vector3d::Zero(), 
        xy_active );
  if(mu){
    toe_eval->set_frictional(); toe_eval->set_mu(mu);
  }
  return toe_eval;
}

template <typename T>
std::tuple<
                std::vector<std::unique_ptr<DirconMode<T>>>,
                std::vector<std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>>> ,
                std::vector<std::unique_ptr<dairlib::multibody::KinematicEvaluatorSet<T>>>
          > createSpiritModeSequence( 
          MultibodyPlant<T>& plant, // multibodyPlant
          Eigen::Matrix<bool,-1,4> modeSeqMat, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
          Eigen::VectorXi knotpointMat, // Matrix of knot points for each mode  
          double mu ){

  std::cout<<modeSeqMat<<std::endl;
  std::cout<<knotpointMat<<std::endl;  

  const double toeRadius = 0.02;
  const Vector3d toeOffset(toeRadius,0,0); // vector to "contact point"
  
  assert( modeSeqMat.rows()==knotpointMat.rows() );

  std::vector<std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>>> toeEvals;
  std::vector<std::unique_ptr<dairlib::multibody::KinematicEvaluatorSet<T>>> toeEvalSets;
  std::vector<std::unique_ptr<DirconMode<T>>> modeVector;
  // DirconModeSequence<T> sequence = DirconModeSequence<T>(plant);

  for (int iMode = 0; iMode<modeSeqMat.rows(); iMode++)
  {
    
    toeEvalSets.push_back( std::move( std::make_unique<dairlib::multibody::KinematicEvaluatorSet<T>>(plant) ));
    for ( int iLeg = 0; iLeg < 4; iLeg++ ){
      if (modeSeqMat(iMode,iLeg)){
        toeEvals.push_back( std::move( getSpiritToeEvaluator(plant, iLeg, toeOffset, mu )) );//Default Normal (z) and xy_active=true
        (toeEvalSets.back())->add_evaluator(  (toeEvals.back()).get()  ); //add evaluator to the set if active //Works ish
      }
    }
    auto dumbToeEvalPtr = (toeEvalSets.back()).get() ;
    int num_knotpoints = knotpointMat(iMode);
    modeVector.push_back(std::move( std::make_unique<DirconMode<T>>( *dumbToeEvalPtr , num_knotpoints )));
    // DirconMode<T> modeDum = DirconMode<T>( *dumbToeEvalPtr , num_knotpoints );
    // sequence.AddMode(  &modeDum  ); // Add the evaluator set to the mode sequence
    
  }
  
  return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
  // return {std::move(toeEvals), std::move(toeEvalSets)};
}
// //Overload function to allow the use of a equal number of knotpoints for every mode.
// template <typename T>
// std::tuple<
//                 DirconModeSequence<T>,
//                 std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
//                 std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>
//           > createSpiritModeSequence( 
//   MultibodyPlant<T>& plant, // multibodyPlant
//   Eigen::Matrix<bool,-1,4> modeSeqMat, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
//   uint16_t knotpoints, // Number of knot points per mode
//   double mu = 1){
//   int numModes = modeSeqMat.rows(); 
//   Eigen::VectorXi knotpointMat = Eigen::MatrixXi::Constant(numModes,1,knotpoints);
//   return createSpiritModeSequence(plant, modeSeqMat,knotpointMat, mu);
// }


template void nominalSpiritStand(
    drake::multibody::MultibodyPlant<double>& plant, 
    Eigen::VectorXd& xState, 
    double height); // NOLINT 
    
template const drake::multibody::Frame<double>& getSpiritToeFrame( 
    drake::multibody::MultibodyPlant<double>& plant, 
    u_int8_t toeIndex ); // NOLINT 

template std::unique_ptr<multibody::WorldPointEvaluator<double>> getSpiritToeEvaluator( 
                      drake::multibody::MultibodyPlant<double>& plant, 
                      u_int8_t toeIndex,
                      const Eigen::Vector3d toePoint,
                      double mu ,
                      const Eigen::Vector3d normal,
                      bool xy_active
                      ); // NOLINT
                      
template std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<double>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<double>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<double>>>   
          >     
    createSpiritModeSequence( 
          drake::multibody::MultibodyPlant<double>& plant, // multibodyPlant
          Eigen::Matrix<bool,-1,4> modeSeqMat, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
          Eigen::VectorXi knotpointMat, // Matrix of knot points for each mode  
          double mu); // NOLINT
  
}//namespace dairlib