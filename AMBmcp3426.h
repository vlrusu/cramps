#ifndef _AMBMCP3426_SOFT_H
#define _AMBMCP3426_SOFT_H

#include <stdint.h>
#include "MI2C.h"

//#define AMBMCP3426_FS 32768
#define AMBMCP3426_FS 8192
//#define AMBMCP3426_FS 4096
#define AMBMCP3426_REFV 2.048
#define AMBMCP3426_ADDRESS 0x68

typedef struct {
  int config;
  MI2C*  _mi2c;
  
} AMBmcp3426_t;


void _AMBmcp3426_init(AMBmcp3426_t* self, MI2C* i2c);
uint16_t _AMBmcp3426_setconfig(AMBmcp3426_t* self, int channel);
uint16_t _AMBmcp3426_read(AMBmcp3426_t* self, float*);

#endif
