#pragma once

#include "u6.h"

namespace labjack {
static constexpr int lj_LocalID = -1;
static constexpr double pole_enc_ppr = 10000;

LABJACK_HANDLE OpenLabjack();

long ConfigureLabjackEncoder(LABJACK_HANDLE handle);

long ResetEncoderAngle(LABJACK_HANDLE handle);

double GetEncoderAngle(LABJACK_HANDLE handle, long* error_code);



}
