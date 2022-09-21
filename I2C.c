/* Arduino I2cMaster Library
 * Copyright (C) 2010 by William Greiman
 *
 * This file is part of the Arduino I2cMaster Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino I2cMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "I2C.h"
#include <unistd.h>

void I2C_setup(I2C *self, MCP *mcp_sda, uint8_t sdaPin, MCP *mcp_scl, uint8_t sclPin)
{
    self->_mcp_sda = mcp_sda;
    self->_sdaPin = sdaPin;
    self->_mcp_scl = mcp_scl;
    self->_sclPin = sclPin;

    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);

    MCP_pinMode(self->_mcp_scl, self->_sclPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);

    /* printf("%d %d\n",self->_sdaPin,self->_sclPin); */
    /* printf("%x %x\n",self->_mcp_sda->_address,self->_mcp_scl->_address); */
    
    /* exit(1); */
}

uint8_t I2C_read(I2C *self, uint8_t last)
{
    uint8_t b = 0;
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_INPUT);
    // read byte
    uint8_t i;
    for (i = 0; i < 8; i++) {
      //       b <<= 1;
        usleep(I2C_DELAY_USEC);
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
	b = (b << 1) | MCP_pinRead(self->_mcp_sda, self->_sdaPin) ;
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    }
    // send Ack or Nak
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, last);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    usleep(I2C_DELAY_USEC);

    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    return b;
}

uint8_t I2C_start(I2C *self, uint8_t addressRW) {
  MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
  MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
	usleep(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    usleep(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    return I2C_write(self, addressRW);
}


void I2C_stop(I2C *self) {
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    usleep(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    usleep(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
    usleep(I2C_DELAY_USEC);
}


uint8_t I2C_write(I2C *self, uint8_t data) {
    // write byte
    uint8_t m;
    for (m = 0X80; m != 0; m >>= 1) {
        MCP_pinWrite(self->_mcp_sda, self->_sdaPin, m & data);
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
        usleep(I2C_DELAY_USEC);
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    }

    // get Ack or Nak
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_INPUT);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    uint8_t rtn = MCP_pinRead(self->_mcp_sda, self->_sdaPin);

    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    return 0;
}

