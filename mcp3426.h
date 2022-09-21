#ifndef _MCP3426_SOFT_H
#define _MCP3426_SOFT_H

#include <stdint.h>
#include "I2C.h"

#define MCP3426_FS 32768
#define MCP3426_REFV 2.048
#define MCP3426_ADDRESS 0x68

typedef struct {
  int scl;
  int sda;
  int config;
  I2C*  _i2c;
  
} mcp3426_t;


void _mcp3426_init(mcp3426_t* self, I2C* i2c);
int _mcp3426_setconfig(mcp3426_t* self, int channel);
float _mcp3426_read(mcp3426_t* self);

#endif
