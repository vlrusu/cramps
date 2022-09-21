#include <stdint.h>
#include <stdio.h>

#include "mcp3426.h"

#include <unistd.h>

void _mcp3426_init(mcp3426_t* self, I2C* i2c){

  self->_i2c = i2c;

  //ocbit not set 0x80 no effect on RDY bit continuous conversion mode
  uint8_t gain = 0; //default x1
  uint8_t samplerate = 2;//15SPS 16 bit
  uint8_t mode = 1;// 1 - continuous conversion 0 - one-shot
  uint8_t channel = 0; //obviously this will be changed by the code
  self->config = 0x80 | (channel << 5) | (mode << 4) | (samplerate << 2) | (gain) ;


}

int _mcp3426_setconfig(mcp3426_t* self, int channel){

  self->config = self->config & ~(1 << 5) | (channel << 5);
  //  printf (" Config %0xh\n", self->config);
	
  if (I2C_start(self->_i2c, MCP3426_ADDRESS << 1 | I2C_WRITE) != I2C_ACK) {fprintf(stderr, "This No device found at address %0xh\n", MCP3426_ADDRESS); return -1;}
  if (I2C_write(self->_i2c, self->config) != I2C_ACK) {fprintf(stderr, "That No device found at address %0xh\n", MCP3426_ADDRESS); return -1;}
  I2C_stop(self->_i2c);
  return 0;
}


float _mcp3426_read(mcp3426_t* self){

  float val;
  uint8_t firstbyte=0;
  uint8_t secondbyte=0;
  uint8_t thirdbyte;


  //  check the conversion ready bit
  while (1){
    if (I2C_start(self->_i2c, MCP3426_ADDRESS << 1 | I2C_READ) != I2C_ACK) fprintf(stderr, "Repeated No device found at address %0xh\n", MCP3426_ADDRESS);
    firstbyte = I2C_read(self->_i2c,0);

    secondbyte = I2C_read(self->_i2c,0);

    thirdbyte = I2C_read(self->_i2c,1);

    /* printf (" Firstbyte %0xh\n", firstbyte); */
    /* printf (" Secondbyte %0xh\n", secondbyte); */
    /* printf (" Thirdbyte %0xh\n", thirdbyte); */
    /* printf (" Test %0xh\n", self->config); */


    /*
     */
    


    if ( (thirdbyte & 0x7F) != (self->config & 0x7F) ) {
      fprintf(stderr, "Bad address in config %0xh should be %0xh\n", thirdbyte, self->config);
      return 0;
    }

    
    if ( (thirdbyte & 0x80)== 0) {break;}
      /* else { */
      /* 	printf("WAITING\n"); */
      /* 	printf (" Firstbyte %0xh\n", firstbyte); */
      /* 	printf (" Secondbyte %0xh\n", secondbyte); */
      /* 	printf (" Thirdbyte %0xh\n", thirdbyte); */
      /* 	printf (" Test %0xh\n", self->config); */
      /* } */
    //      usleep(1000);
    //    }
    
    I2C_stop(self->_i2c);
    }



  val = firstbyte*256. + secondbyte;

  if (val > MCP3426_FS-1)
    val = -MCP3426_REFV+(val-MCP3426_FS-1)*MCP3426_REFV/MCP3426_FS;
  else
    val = MCP3426_REFV*val/MCP3426_FS;


  return val;


}
