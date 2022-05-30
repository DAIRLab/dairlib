#include "examples/franka/joint_impedance_controller.h"

JointImpedanceController::JointImpedanceController(const MatrixXd& K, const MatrixXd& B){
    // TODO: add in exception throwing if sizes don't match
    K_ = K;
    B_ = B;

    // set up input ports
    // TODO: fill in port arguments
    input_index_state_ = 
        this->DeclareVectorInputPort("current_state", 14).get_index(); // TODO: is this 7 or 14?
    
    // input_index_desired_state_ = 
    //     this->DeclareVectorInputPort("desired_state", ).get_index();
    // input_index_lambda_= 
    //     this->DeclareVectorInputPort("lambda_desired", ).get_index();

    // set up output port
    output_index_control_ = 
        this->DecalreVectorOutputPort("control", ).get_index();
    

}
