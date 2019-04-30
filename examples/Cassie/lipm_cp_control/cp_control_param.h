#pragma once

namespace dairlib {
namespace systems {


const double stanceDurationPerLeg = 0.35;

const bool isDebugAndStandingStill = false; // The CoM Traj is a fixed point. This is used to test operational space control.
const bool isReadTrajOnlyAtTouchDown = false;

const bool isPrintInfo = true; // Print the controller information (debug purpose)





} //namespace systems
} //namespace dairlib


