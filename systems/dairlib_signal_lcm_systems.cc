#include "systems/dairlib_signal_lcm_systems.h"

#include <limits>
typedef std::numeric_limits<double> dbl;

using std::cout;
using std::endl;

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using std::string;

/*--------------------------------------------------------------------------*/
// methods implementation for DairlibSignalReceiver.

DairlibSignalReceiver::DairlibSignalReceiver(int signal_size)
    : signal_size_(signal_size) {
  this->DeclareAbstractInputPort("lcmt_dairlib_signal",
                                 drake::Value<dairlib::lcmt_dairlib_signal>{});
  this->DeclareVectorOutputPort("signal", BasicVector<double>(signal_size),
                                &DairlibSignalReceiver::UnpackLcmIntoVector);
}

void DairlibSignalReceiver::UnpackLcmIntoVector(
    const Context<double>& context, BasicVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<dairlib::lcmt_dairlib_signal>();
  for (int i = 0; i < signal_size_; i++) {
    // We assume that the order of the vector is [data, timestamp]
    output->get_mutable_value()(i) = input_msg.val[i];
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for DrakeSignalSender.

DrakeSignalSender::DrakeSignalSender(
    const std::vector<std::string>& signal_names, double stride_period)
    : signal_names_(signal_names),
      signal_size_(signal_names.size()),
      stride_period_(stride_period) {
  this->DeclareVectorInputPort("signal", BasicVector<double>(signal_names.size()));
  this->DeclareAbstractOutputPort("signal", &DrakeSignalSender::PackVectorIntoLcm);
}

void DrakeSignalSender::PackVectorIntoLcm(
    const Context<double>& context, dairlib::lcmt_dairlib_signal* msg) const {
  const auto* input_vector = this->EvalVectorInput(context, 0);

  msg->dim = signal_size_;
  msg->val.resize(signal_size_);
  msg->coord.resize(signal_size_);
  for (int i = 0; i < signal_size_; i++) {
    msg->val[i] = input_vector->get_value()(i);
    msg->coord[i] = signal_names_[i];
  }
  // Add epsilon to avoid error from converting double to int.
  msg->utime = (context.get_time() + 1e-12) * 1e6;

  // Testing -- Calc phase
//  double lift_off_time = input_vector->get_value()(1);
//  double time_in_first_mode = (msg->utime * 1e-6) - lift_off_time;
//  double init_phase = time_in_first_mode / stride_period_;

  /*cout << "init_phase = " << init_phase<<"\n";
  cout << "fsm state = " << input_vector->get_value()(0) << endl;
  cout << "lift_off_time = " << lift_off_time << endl;
  cout << "current_time = " << context.get_time() << endl;
  cout << "time_in_first_mode = " << time_in_first_mode << endl;
  cout << "input_vector->get_value() = " << input_vector->get_value() << endl;*/

  /*
  if (init_phase > 1) {
    cout.precision(dbl::max_digits10);

    cout << "WARNING: phase = " << init_phase
         << " (>= 1). There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    cout << "fsm state = " << input_vector->get_value()(0) << endl;
    cout << "lift_off_time = " << lift_off_time << endl;
    cout << "current_time = " << context.get_time() << endl;
    cout << "time_in_first_mode = " << time_in_first_mode << endl;
    cout << "input_vector->get_value() = " << input_vector->get_value() << endl;
    cout << '\a';  // making noise to notify
    cout << "======================\n";
    cout << "======================\n";
    cout << "======================\n";
    cout << "======================\n";
    cout << "======================\n";
    //    DRAKE_UNREACHABLE();
  } else if (init_phase < 0) {
    cout.precision(dbl::max_digits10);

    cout << "WARNING: phase = " << init_phase
         << " (<0). There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    cout << "fsm state = " << input_vector->get_value()(0) << endl;
    cout << "lift_off_time = " << lift_off_time << endl;
    cout << "current_time = " << context.get_time() << endl;
    cout << "time_in_first_mode = " << time_in_first_mode << endl;
    cout << "input_vector->get_value() = " << input_vector->get_value() << endl;
    cout << '\a';  // making noise to notify
    cout << "======================\n";
    cout << "======================\n";
    cout << "======================\n";
    cout << "======================\n";
    cout << "======================\n";
    //    DRAKE_UNREACHABLE();
  }*/
}
}  // namespace systems
}  // namespace dairlib
