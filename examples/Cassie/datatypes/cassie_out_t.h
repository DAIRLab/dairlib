#pragma once

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "cassie_out_t_types.h"
#define CASSIE_OUT_T_LEN               697

#ifdef __cplusplus
extern "C" {
#endif

extern void cassie_out_t_initialize(void);
extern void cassie_out_t_terminate(void);
extern void pack_cassie_out_t(const cassie_out_t *bus, unsigned char bytes[697]);
extern void unpack_cassie_out_t(const unsigned char bytes[697], cassie_out_t
  *bus);

#ifdef __cplusplus
}
#endif
