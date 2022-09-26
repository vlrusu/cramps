#include <stdint.h>
#include <stdio.h>

#include "AMBmcp3426.h"

#include <unistd.h>

void _AMBmcp3426_init(AMBmcp3426_t* self, MI2C* mi2c){

  self->_mi2c = mi2c;

  //ocbit not set 0x80 no effect on RDY bit continuous conversion mode
  uint8_t gain = 0; //default x1
  uint8_t samplerate = 2;// 16SPS 16 bit
  uint8_t mode = 1;// 1 - continuous conversion 0 - one-shot
  uint8_t channel = 0; //obviously this will be changed by the code
  self->config = 0x80 | (channel << 5) | (mode << 4) | (samplerate << 2) | (gain) ;


}

uint16_t _AMBmcp3426_setconfig(AMBmcp3426_t* self, int channel){

  self->config = self->config & ~(1 << 5) | (channel << 5);
  //  printf (" Config %0xh\n", self->config);

  uint16_t ret = MI2C_start(self->_mi2c, AMBMCP3426_ADDRESS << 1 | MI2C_WRITE);
  
  if (ret != MI2C_ACK) {fprintf(stderr, "This No device found at address %0xh\n", AMBMCP3426_ADDRESS); return ret;}

  ret = MI2C_write(self->_mi2c, self->config);
    
  if (ret != MI2C_ACK) {fprintf(stderr, "That No device found at address %0xh\n", AMBMCP3426_ADDRESS); return ret;}
  MI2C_stop(self->_mi2c);
  return 0;
}


uint16_t _AMBmcp3426_read(AMBmcp3426_t* self, float* dataByCramp){


  uint16_t firstByte[8];
  uint16_t secondByte[8];
  uint16_t thirdByte[8];

  //need to add the mask here

  
  uint8_t firstByteByCramp[16];
  uint8_t secondByteByCramp[16];
  uint8_t thirdByteByCramp[16];

  uint8_t nCramps = self->_mi2c->_nCramps;

  //printf("nCramps=%d\n",nCramps);
  uint16_t retc = 0;

  //  check the conversion ready bit



	
    retc = MI2C_start(self->_mi2c, AMBMCP3426_ADDRESS << 1 | MI2C_READ);


    if (retc != MI2C_ACK) {fprintf(stderr, "Repeated No device found at address %0xh %x\n", AMBMCP3426_ADDRESS, retc);}
    MI2C_read(self->_mi2c,0,&firstByte);

    /* for (int i =0; i < 8; i++) */
    /*   printf("%d firstByte[%d] = %x\n",retc,i,firstByte[i]); */
    
    MI2C_read(self->_mi2c,0,&secondByte);


    //keep checking the RDY bit to make sure the next conversion is ready
    while (1){
    
      MI2C_read(self->_mi2c,0,&thirdByte);

    
    /* printf (" Firstbyte %0xh\n", firstbyte); */
    /* printf (" Secondbyte %0xh\n", secondbyte); */
    /* printf (" Thirdbyte %0xh\n", thirdbyte); */
    /* printf (" Test %0xh\n", self->config); */


    /*
     */

    //rearrange by cramp
    for (uint8_t i = 0; i<8; i++) {
      uint16_t port1 = firstByte[i];
      uint16_t port2 = secondByte[i];
      uint16_t port3 = thirdByte[i];
      uint8_t* pFirstByteByCramp = (uint8_t*)firstByteByCramp;
      uint8_t* pSecondByteByCramp = (uint8_t*)secondByteByCramp;
      uint8_t* pThirdByteByCramp = (uint8_t*)thirdByteByCramp;
      for (uint8_t chn = 0; chn<nCramps; chn++) {
	pFirstByteByCramp[0] <<= 1;
	pFirstByteByCramp[0] |= port1 & 1;
	port1 >>= 1;
	pSecondByteByCramp[0] <<= 1;
	pSecondByteByCramp[0] |= port2 & 1;
	port2 >>= 1;
	pThirdByteByCramp[0] <<= 1;
	pThirdByteByCramp[0] |= port3 & 1;
	port3 >>= 1;
	pFirstByteByCramp++;
	pSecondByteByCramp++;
	pThirdByteByCramp++;	
      }
    }

    //    
    for (uint8_t chn = 0; chn<nCramps; chn++) {
      if ( (thirdByteByCramp[chn] & 0x7F) != (self->config & 0x7F) ) {
	fprintf(stderr, "Bad address in location %d config %0xh should be %0xh\n", chn, thirdByteByCramp[chn], self->config);
	return 0;
      }
    }

    //    printf("%x %x \n",thirdByteByCramp[0],self->config);
    //    printf("%x %x \n",thirdByteByCramp[1],self->config);


    int countReady = 0;
    for (uint8_t chn = 0; chn<nCramps; chn++) {
      if ((thirdByteByCramp[chn] & 0x80 )== 0) countReady++;

    }

    usleep(5000); //don't know why I need this here, but w/o a delay.the RDY bit above never gets set.
    //    printf("countRead=%d %d\n",countReady,nCramps);

    //    exit(1);

    if ( countReady == nCramps) break; //check this logic
    /* else { */
      /* 	printf("WAITING\n"); */
      /* 	printf (" Firstbyte %0xh\n", firstbyte); */
      /* 	printf (" Secondbyte %0xh\n", secondbyte); */
      /* 	printf (" Thirdbyte %0xh\n", thirdbyte); */
      /* 	printf (" Test %0xh\n", self->config); */
      /* } */
    //      usleep(1000);
    //    }
    

    }

    MI2C_read(self->_mi2c,1,&thirdByte);    
    MI2C_stop(self->_mi2c);
    
  
  for (uint8_t chn = 0; chn<nCramps; chn++) {
    uint16_t rawdata = (firstByteByCramp[chn]<<8) + secondByteByCramp[chn];
    int16_t val = (int16_t) (rawdata);

    dataByCramp[chn]  = AMBMCP3426_REFV*(float)val/AMBMCP3426_FS;
      

  }

  return retc;


}
