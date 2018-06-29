#include "systems/controllers/affine_controller.h"

namespace dairlib{
namespace systems{

AffineController::AffineController(int num_positions, int num_velocities, int num_efforts):
    num_states_(num_positions + num_velocities), num_efforts_(num_efforts)
{

  input_port_info_index_ = this->DeclareVectorInputPort(
      OutputVector<double>(num_positions, num_velocities,
                           num_efforts)).get_index();

  input_port_params_index_ = this->DeclareVectorInputPort(
      AffineParams(num_states_, num_efforts_)).get_index();

  this->DeclareVectorOutputPort(TimestampedVector<double>(num_efforts_),
      &AffineController::CalcControl);
  
}


MatrixXd AffineController::VecToMat(VectorXd v, int num_rows, int num_cols) const
    {
        MatrixXd m = MatrixXd::Zero(num_rows, num_cols);
        for(int i=0; i<num_cols; i++)
        {
            VectorXd vcol = v.segment(i*num_rows, num_rows) ;
            m.col(i) = vcol;
        }

        return m;

    }

VectorXd AffineController::MatToVec(MatrixXd m) const
{
    int num_rows = m.rows();
    int num_cols = m.cols();
    VectorXd v = VectorXd::Zero(num_rows*num_cols);

    for(int i=0; i<num_cols; i++)
    {
        v.segment(i*num_rows, num_rows) = m.col(i);
    }

    return v;
}


void AffineController::CalcControl(const Context<double>& context,
                                  TimestampedVector<double>* control) const
{
    const OutputVector<double>* info = (OutputVector<double>*)
        this->EvalVectorInput(context, input_port_info_index_);

    const AffineParams* params = dynamic_cast<const AffineParams*>(
        this->EvalVectorInput(context, input_port_params_index_));


    VectorXd params_vec = params->CopyVectorNoTimestamp();
    VectorXd K_vec = params_vec.head(num_states_*num_efforts_);
    VectorXd C = params_vec.segment(num_states_*num_efforts_, num_efforts_);
    VectorXd desired_state = params_vec.tail(num_states_);
    MatrixXd K = VecToMat(K_vec, num_efforts_, num_states_);

    VectorXd u = K*(desired_state - info->GetState()) + C;

    control->SetDataVector(u);
    control->set_timestamp(info->get_timestamp());
    
    }


}// namespace systems
}// namespace dairlib
