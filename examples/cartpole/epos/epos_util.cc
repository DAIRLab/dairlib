#include "epos_util.h"

namespace epos {

MAXON_HANDLE OpenDevice(unsigned int *error_code) {
  char *pDeviceName = new char[255];
  char *pProtocolStackName = new char[255];
  char *pInterfaceName = new char[255];
  char *pPortName = new char[255];

  MAXON_HANDLE handle = nullptr;
  strcpy(pDeviceName, ep_DeviceName);
  strcpy(pProtocolStackName, ep_ProtocolStackName);
  strcpy(pInterfaceName, ep_InterfaceName);
  strcpy(pPortName, ep_PortName);

  handle = VCS_OpenDevice(
      pDeviceName, pProtocolStackName, pInterfaceName, pPortName, error_code);

  if (handle != nullptr && *error_code == 0) {
    unsigned int baudrate = 0;
    unsigned int timeout = 0;
    VCS_GetProtocolStackSettings(
        handle, &baudrate, &timeout, error_code);
    VCS_SetProtocolStackSettings(
        handle, ep_BaudRate, timeout, error_code);
  } else {
    handle = nullptr;
  }

  delete[]pDeviceName;
  delete[]pProtocolStackName;
  delete[]pInterfaceName;
  delete[]pPortName;

  return handle;
}

unsigned int CloseDevice(MAXON_HANDLE handle) {
  unsigned int error_code = 0;
  VCS_CloseDevice(handle, &error_code);
  return error_code;
}

unsigned int EnableDevice(MAXON_HANDLE handle) {
  int isFault = 0;
  int lResult = 0;
  unsigned int p_pErrorCode = 0;

  if (VCS_GetFaultState(handle, ep_NodeId, &isFault, &p_pErrorCode) == 0) {
    LogError("VCS_GetFaultState", lResult, p_pErrorCode);
  }

  if (isFault) {
    if (VCS_ClearFault(handle, ep_NodeId, &p_pErrorCode) == 0) {
      LogError("VCS_ClearFault", lResult, p_pErrorCode);
      lResult = 1;
    }
  }

  if (lResult == 0) {
    int oIsEnabled = 0;

    if (VCS_GetEnableState(handle, ep_NodeId, &oIsEnabled, &p_pErrorCode)
        == 0) {
      LogError("VCS_GetEnableState", lResult, p_pErrorCode);
      lResult = 1;
    }

    if (lResult == 0) {
      if (!oIsEnabled) {
        if (VCS_SetEnableState(handle, ep_NodeId, &p_pErrorCode) == 0) {
          LogError("VCS_SetEnableState", lResult, p_pErrorCode);
          lResult = 1;
        }
      }
    }
  }
  return p_pErrorCode;
}

double map_motor_to_position(int enc_pos) {
  double enc_pos_gain = -4.72977409598073e-6;
  return  enc_pos * enc_pos_gain;
}

int make_desired_current(double force) {
  double current = -force / 0.0075;
  if (current > 7000.0) {
    return 7000;
  } else if (current < -7000.0) {
    return -7000;
  } else {
    return (int) current;
  }
}

double make_current_force(int current) {
  double force = - (double) current * 0.0075;
  return force;
}

unsigned int HomeDevice(MAXON_HANDLE handle) {
  unsigned int activate_err = 0;
  VCS_SetOperationMode(handle, ep_NodeId, OMD_HOMING_MODE, &activate_err);
  VCS_FindHome(handle, ep_NodeId, HM_ACTUAL_POSITION, &activate_err);
  VCS_StopHoming(handle, ep_NodeId, &activate_err);

  return activate_err;
}

unsigned int SetCurrentControlMode(MAXON_HANDLE handle) {
  unsigned int activate_err = 0;
  VCS_SetOperationMode(handle, ep_NodeId, OMD_CURRENT_MODE, &activate_err);
  return activate_err;
}

unsigned int SetCurrentByForce(MAXON_HANDLE handle, double force) {
  unsigned int activate_err = 0;
  int current = make_desired_current(force);
  VCS_SetCurrentMustEx(handle, ep_NodeId, current, &activate_err);
  return activate_err;
}

double GetForceFromCurrent(MAXON_HANDLE handle) {
  unsigned int activate_err = 0;
  int measured_current = 0;
  VCS_GetCurrentIsEx(handle, ep_NodeId, &measured_current, &activate_err);
  return make_current_force(measured_current);
}

double GetCartPosition(MAXON_HANDLE handle) {
  unsigned int activate_err = 0;
  int enc_pos = 0;
  VCS_GetPositionIs(handle, ep_NodeId, &enc_pos, &activate_err);
  return map_motor_to_position(enc_pos);
}

};