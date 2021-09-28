#include "labjack_utils.h"

namespace labjack {

LABJACK_HANDLE OpenLabjack() {
  return openUSBConnection(lj_LocalID);
}

long ConfigureLabjackEncoder(LABJACK_HANDLE handle) {
  u6CalibrationInfo caliInfo;
  long error;
  long alngEnableTimers[4] = {1, 1, 0, 0};
  long alngTimerModes[4] = {LJ_tmQUAD, LJ_tmQUAD, 0, 0};
  long alngEnableCounters[2] = {0,1};
  double adblTimerValues[4] = {0, 0 ,0, 0};

  error = eTCConfig(handle, alngEnableTimers, alngEnableTimers, 0,
      LJ_tc48MHZ, 2, alngTimerModes, adblTimerValues, 0, 0);
  return error;
}

double GetEncoderAngle(LABJACK_HANDLE handle, long *error_code) {
  long aReadTimers[4] = {1, 0, 0, 0};
  long aUpdateResetTimers[4] = {0, 0, 0, 0};
  long aReadCounters[2] = {0,0};
  long aResetCounters[2] = {0, 0};
  double aTimerValues[4] = {0, 0, 0, 0};
  double aCounterValues[2] = {0, 0};
  *error_code = eTCValues(
      handle,
      aReadTimers,
      aUpdateResetTimers,
      aReadCounters,
      aResetCounters,
      aTimerValues,
      aCounterValues, 0, 0);
  int timer_vals[4] = reinterpret_cast<int>(aTimerValues);
  int enc_pos = 0;
  return M_2_PI * (double)enc_pos / pole_enc_ppr;
}



}