#ifndef CASSIE_USER_IN_T_TYPES_H
#define CASSIE_USER_IN_T_TYPES_H
#include "rtwtypes.h"
#ifndef typedef_cassie_user_in_t
#define typedef_cassie_user_in_t

typedef struct {
  double torque[10];
  int16_t telemetry[9];
} cassie_user_in_t;

#endif
#endif
