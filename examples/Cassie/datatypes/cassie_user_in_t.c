#include "cassie_user_in_t.h"
#include <stddef.h>

void cassie_user_in_t_initialize(void)
{
}

void cassie_user_in_t_terminate(void)
{

}

void pack_cassie_user_in_t(const cassie_user_in_t *bus, unsigned char bytes[58])
{
  int i0;
  unsigned char y[40];
  float x[10];
  unsigned char b_y[18];
  short b_x[9];
  for (i0 = 0; i0 < 10; i0++) {
    x[i0] = (float)bus->torque[i0];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)40 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 9; i0++) {
    b_x[i0] = bus->telemetry[i0];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)18 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 40; i0++) {
    bytes[i0] = y[i0];
  }

  for (i0 = 0; i0 < 18; i0++) {
    bytes[i0 + 40] = b_y[i0];
  }
}

void unpack_cassie_user_in_t(const unsigned char bytes[58], cassie_user_in_t
  *bus)
{
  int i;
  float y[10];
  unsigned char x[40];
  unsigned char b_x[18];
  for (i = 0; i < 40; i++) {
    x[i] = bytes[i];
  }

  memcpy((void *)&y[0], (void *)&x[0], (unsigned int)((size_t)10 * sizeof(float)));
  for (i = 0; i < 10; i++) {
    bus->torque[i] = y[i];
  }

  for (i = 0; i < 18; i++) {
    b_x[i] = bytes[i + 40];
  }

  memcpy((void *)&bus->telemetry[0], (void *)&b_x[0], (unsigned int)((size_t)9 *
          sizeof(short)));
}
