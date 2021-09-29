#pragma once

#include "Definitions.h"
#include <string>
#include <iostream>
#include <cstring>

typedef void* MAXON_HANDLE;

using std::string;

namespace epos {

static constexpr unsigned short ep_NodeId = 1;
static constexpr char ep_DeviceName[] = "EPOS4";
static constexpr char  ep_ProtocolStackName[] = "MAXON SERIAL V2";
static constexpr char  ep_InterfaceName[] = "USB";
static constexpr char  ep_PortName[] = "USB0";
static constexpr int ep_BaudRate = 1000000;

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);

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

