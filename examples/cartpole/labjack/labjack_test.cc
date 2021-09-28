#include "labjack_utils.h"
#include <iostream>

int main(int argc, char* argv[]) {
  LABJACK_HANDLE handle = labjack::OpenLabjack();
  labjack::ConfigureLabjackEncoder(handle);
  long error_code = 0;
  while (true) {
    std::cout << "Encoder angle: " <<
    labjack::GetEncoderAngle(handle, &error_code) << std::endl;
  }
  return 0;
}