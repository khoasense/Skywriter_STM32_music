#ifndef _SKYWRITER_INCL
#define _SKYWRITER_INCL
#include "stdint.h"

typedef enum
{
  PACKET_NOTHING = 0,
  PACKET_TOUCH = 1 << 0,
  PACKET_AIRWHEEL  = 1 << 1,
  PACKET_GESTURE =  1 << 2,
  PACKET_XYZ = 1 << 3
} packetType_t;

void skywriter_init(uint16_t xferPin, uint16_t resetPin);
packetType_t skywriter_poll();
void getXYZ(unsigned int *x_out, unsigned int *y_out, unsigned int *z_out);
unsigned char getGesture();
int getRotation();
#else
#endif