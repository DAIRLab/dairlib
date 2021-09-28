#pragma once

#include "Definitions.h"
#include <string>
#include <iostream>
#include <cstring>

typedef void* MAXON_HANDLE;

using std::string;

namespace epos {

unsigned short ep_NodeId = 1;
static const char ep_DeviceName[] = "EPOS4";
static const char  ep_ProtocolStackName[] = "MAXON SERIAL V2";
static const char  ep_InterfaceName[] = "USB";
static const char  ep_PortName[] = "USB0";
int ep_BaudRate = 1000000;

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
  std::cerr << "Error in EPOS function: "<<
  functionName << " failed (result=" << p_lResult <<
  ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

MAXON_HANDLE OpenDevice(unsigned int *error_code);
unsigned int CloseDevice(MAXON_HANDLE handle);
unsigned int EnableDevice(MAXON_HANDLE handle);
unsigned int SetCurrentControlMode(MAXON_HANDLE handle);
unsigned int SetCurrentByForce(MAXON_HANDLE handle, double force);
double GetForceFromCurrent(MAXON_HANDLE handle);
double GetCartPosition(MAXON_HANDLE handle);


double map_motor_to_position(int enc_pos);
int make_desired_current(double force);
double make_current_force(int current);

unsigned int HomeDevice(MAXON_HANDLE handle);

}

