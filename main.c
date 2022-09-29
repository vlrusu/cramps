#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>


#include <softPwm.h>
#include <linux/spi/spidev.h>
#include "SPI_daisy.h"
#include "HDC2080.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "MCP23S17.h"
#include "AMBmcp3426.h"
#include <signal.h>
#include <wiringPi.h>

#define MCPPINBASE 100
#define BMESCLK 2
#define BMEMISO 3
#define BMEMOSI 1
#define BMECS  0

#define HDCSCL 6
#define HDCSDA 5


#define NSTEPS 200
#define SPICS 1
//#define SPISPEED 320000


SPI_daisy spi_sensor;

HDC2080 hdc;
MCP sensorMCP;
I2C i2c_sensor;
struct bme280_dev ptscal;

static volatile int keepRunning = 1;


enum MCPs{MCPHV0, MCPHV1, MCPHV2, MCPHV3};
MCP crampMCP[4];
MI2C mi2c_cramps[4];

#define MAX_LINE_LENGTH 24

int SlotCounter = 0;

AMBmcp3426_t adc[4];
int enabledSlots[MAX_LINE_LENGTH];


void intHandler(int dummy) {
  keepRunning = 0;
}

void readInput(){
  char *path;
  char line[MAX_LINE_LENGTH] = {0};

  // read enabled channels from input.txt
  path = "input.txt";

  FILE *file = fopen(path, "r");
  for (int i = 0; i < MAX_LINE_LENGTH; i++) {
    enabledSlots[i] = -1;
  }
  while(fgets(line, MAX_LINE_LENGTH, file)){
    enabledSlots[SlotCounter] = atoi(line);
    SlotCounter++;
  }
  fclose(file);
}

void recover(){


  printf("Recovering\n");
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);

  sleep(1);

  digitalWrite(10,LOW);

  sleep(1);

  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);

  
}


int mygetch ( void )
{
  int ch;
  struct termios oldt, newt;

  tcgetattr ( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr ( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr ( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

void initialization(int crampchannel){

  //setup MCP

  //  void MCP_setup( MCP* mcp, uint8_t spibus, uint8_t ss, uint8_t address);
  /* MCP_setup (&sensorMCP, 1, 1, 0); */

  /* //  printf("mcp setup done\n"); */

  /* I2C_setup(&i2c_sensor, &sensorMCP, 5,&sensorMCP, 6); */
  
  /* hdc2080_setup(&hdc, HDC2080_I2C_ADDR_PRIM, &i2c_sensor ); */

  /* //SPI_daisy bus for BME */
  /* SPI_daisy_setup(&spi_sensor, &sensorMCP, BMESCLK, &sensorMCP, BMEMOSI, &sensorMCP, BMEMISO,&sensorMCP, BMECS); */




  /* ptscal.dev_id = 0; */
  /* ptscal.intf = BME280_SPI_INTF; */
  /* ptscal._spidaisy = &spi_sensor; */
  /* ptscal.delay_ms = delay; */

  /* bme280_init(&ptscal); */

  /* uint8_t ptscalchipid = 0; */
  /* bme280_get_regs(BME280_CHIP_ID_ADDR,&ptscalchipid,1,&ptscal); */


  //setup MCPs for cramps
  for (int imcp = MCPHV0; imcp<=MCPHV3; imcp++){
    MCP_setup (&crampMCP[imcp], 0, 0, 0x20 + imcp );
  }



  /* uint8_t which = MCPHV3;  */
  /* uint8_t pin = 8; */
  /* MCP_pinMode(&crampMCP[which], pin, MCP_OUTPUT); */
  /* MCP_pinWrite(&crampMCP[which], pin, 1); */


  /*   uint8_t value = 0x0; */
  /*   MCP_byteWrite(&crampMCP[which], IODIRA, value); */
  /*   MCP_byteWrite(&crampMCP[which], IODIRB, value); */


  /*   value = 0x0;   */
  /*   MCP_byteWrite(&crampMCP[which], GPIOA, value); */
  /*   MCP_byteWrite(&crampMCP[which], GPIOB, value); */
  /*   value = 0; */

  /*   value = MCP_byteRead(&crampMCP[which], GPIOA); */
  /* printf("VAL1=%x\n",value); */
  /*   value = MCP_byteRead(&crampMCP[which], GPIOB); */


  //  printf("VAL2=%x\n",value);
  //  exit(1);



  uint16_t crampMasks[4]= {0,0,0,0};
  //initialize cramps
  for (int i = 0; i < SlotCounter; i++) {

      printf("Enabling slot %d\n", enabledSlots[i]);

      
      if (enabledSlots[i]>=0 && enabledSlots[i] <14)
	crampMasks[0] |= (1<< (enabledSlots[i]+2));
      if (enabledSlots[i]>=14 && enabledSlots[i] <24)
	crampMasks[1] |= (1<< (enabledSlots[i]-14));
      if (enabledSlots[i]>=24 && enabledSlots[i] <38)
	crampMasks[2] |= (1<< (enabledSlots[i]-24+2));
      if (enabledSlots[i]>=38 && enabledSlots[i] <48)
	crampMasks[3] |= (1<< (enabledSlots[i]-38));
  }
	
  //  printf("crampMask3=%x\n",crampMasks[3]);
  MI2C_setup(&mi2c_cramps[0], &crampMCP[MCPHV0], crampMasks[0], &crampMCP[MCPHV0], 1); //SDA SCL - this is new
  MI2C_setup(&mi2c_cramps[1], &crampMCP[MCPHV1], crampMasks[1], &crampMCP[MCPHV0], 1); //SDA SCL - this is new
  MI2C_setup(&mi2c_cramps[2], &crampMCP[MCPHV2], crampMasks[2], &crampMCP[MCPHV2], 1); //SDA SCL - this is new
  MI2C_setup(&mi2c_cramps[3], &crampMCP[MCPHV3], crampMasks[3], &crampMCP[MCPHV2], 1); //SDA SCL - this is new  

  for (int i = 0 ; i < 4; i++){
    _AMBmcp3426_init(&adc[i], &mi2c_cramps[i]);
  }



  for (int i = 0; i< 4 ; i++){
    _AMBmcp3426_setconfig(&adc[i],crampchannel);
  }

  
  printf("done init\n");
}



int main(int argc, char *argv[])
{

  int channelnumber = atoi(argv[1]);
  //  FILE *pFile;
  //  pFile=fopen("htp_2.txt", "a+");
  readInput();
  initialization(channelnumber);


  wiringPiSetup();


  char * outputFilename = "currvstime.csv";
  FILE *fpt;
  fpt = fopen(outputFilename, "w+"); //output file
   
  //  sleep(1);
  //  int channel = atoi(argv[1]);


  int n = 0;

  int countloop =0 ;

  while (keepRunning ){
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    clock_t t;
    t = clock();



    //    fprintf(fpt, "%d-%d-%d %d:%d:%d,", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    //    printf("%s %7.1f\n", p, ts.tv_nsec/1e6);
    //    fprintf(fpt,"%s %7.1f ", p, ts.tv_nsec/1e6);


    float currents[24];
    float currents1[24];
    float tmpcurrents[24];
    float tmpcurrents1[24];

    uint8_t ncramps = 0;
    int tmp = 0;
    uint8_t currentArrayIndex = 0;


    //first do the 0 channels
    for (int i = 0; i< 4 ; i++){    
      ncramps = adc[i]._mi2c->_nCramps;
      //      uint8_t retc = 0;
      uint8_t retc = _AMBmcp3426_read(&adc[i], &tmpcurrents);
      if (retc==9){

	recover();
	printf("Recover on error\n");
	usleep(100);
	initialization(channelnumber);
	continue;
	//	exit(1);
      }
      memcpy(currents+currentArrayIndex, tmpcurrents, sizeof(float)*ncramps );
      currentArrayIndex += ncramps;
    }


     //currentArrayIndex = 0; 
     //for (int i = 0; i< 4 ; i++){ 
     //  tmp =  _AMBmcp3426_setconfig(&adc[i],1); 
     //} 

     ////then do the 1 channels */
     //for (int i = 0; i< 4 ; i++){ 
     //  ncramps = adc[i]._mi2c->_nCramps; 
     //  _AMBmcp3426_read(&adc[i], &tmpcurrents1); 
     //  memcpy(currents1+currentArrayIndex, tmpcurrents1, sizeof(float)*ncramps ); 
     //  currentArrayIndex += ncramps; 
     //} 

    struct timeval tv;
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[64], buf[64];
    
    gettimeofday(&tv, NULL);
    nowtime = tv.tv_sec;
    nowtm = localtime(&nowtime);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
    fprintf(fpt,"%s.%06ld", tmbuf, tv.tv_usec);
    fprintf(fpt,",");

    
    for (int i = 0 ; i < currentArrayIndex; i++){
      currents[i] = 1e2*currents[i];
      //      printf(" %8.4f  ",currents[i]);

      fprintf(fpt,"%8.4f,",currents[i]);
      
    }
    //    printf("\n");
    fprintf(fpt,"\n");




    /* for (int i = 0; i < SlotCounter; i++) { */
  
    /*   if (enabledSlots[i] >= 0) { */
    /* 	currents[2*i] = 0; */

    /*     int tmp = 0; */
    /* 	tmp =  _AMBmcp3426_setconfig(&adc[enabledSlots[i]],0); */
    /* 	//	sleep(1); */
    /* 	currents[2*i] = 1e2*_AMBmcp3426_read(&adc[enabledSlots[i]]); */

    /*   } */
    /* } */
    /* //now do the 1 channels */
    /* for (int i = 0; i < SlotCounter; i++) { */
  
    /*   if (enabledSlots[i] >= 0) { */
    /* 	currents[2*i+1] = 0; */

    /*     int tmp = 0; */
    /* 	tmp =  _AMBmcp3426_setconfig(&adc[enabledSlots[i]],0); */
    /* 	//	sleep(1); */
    /* 	currents[2*i+1] = 1e2*_AMBmcp3426_read(&adc[enabledSlots[i]]); */

    /*   } */
    /* } */







    /* for (int i = 0; i < SlotCounter; i++) { */
      
	
    /*         printf("%d %.3f,%.3f\n", enabledSlots[i],currents[2*i], currents[2*i+1]); */
    /*         fprintf(fpt," %.3f,%.3f", currents[2*i], currents[2*i+1]); */

    /* } */
    /* fprintf(fpt,"\n"); */



    struct timespec tend;
    clock_gettime(CLOCK_REALTIME, &tend);

    double seconds = (tend.tv_sec - ts.tv_sec) + (tend.tv_nsec - ts.tv_nsec)/1e9;

    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
 
    //    printf("Entire loop took %f seconds to execute \n", time_taken);

    
    if (countloop%100 == 0)
      printf("%f seconds\n", seconds);

    countloop++;


    //   keepRunning = 0;
    

}
  
  /*
    uint8_t settings_sel;
    struct bme280_data comp_data;

    // Recommended mode of operation: Indoor navigation
    ptscal.settings.osr_h = BME280_OVERSAMPLING_16X;
    ptscal.settings.osr_p = BME280_OVERSAMPLING_16X;
    ptscal.settings.osr_t = BME280_OVERSAMPLING_2X;
    ptscal.settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    int8_t rslt = BME280_OK;
 
    rslt = bme280_set_sensor_settings(settings_sel, &ptscal);

    // float req_delay = bme280_cal_meas_delay(ptscal.settings);
    // printf("delay=%5.2f\n",req_delay);
    ptscal.delay_ms(10);

    signal(SIGINT, intHandler);
    printf("datetime, temp_1, hum_1, temp_2, press, hum_2 \n");
    // while(1 && keepRunning){
    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptscal);
    ptscal.delay_ms(100);
 
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptscal);

    float  temp = comp_data.temperature;
    float  press = 0.01 * comp_data.pressure;
    float  hum = comp_data.humidity;
    //printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);


    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    // printf("%d-%d-%d %d:%d:%d,", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);


 
  
    uint16_t this_temp = 0;
    uint16_t this_humidity = 0;


    rslt = hdc2080_trigger_measurement(&hdc);
    rslt = hdc2080_read_temp(&hdc, &this_temp);
    rslt = hdc2080_read_humidity(&hdc, &this_humidity);

    float temp_hdc = ( (float) this_temp)/65536*165-40;
    float hum_hdc =( (float)this_humidity)/65536*100;

  
    //  printf("HDC %d %d\n",this_temp,this_humidity);
    //  printf("BME %5.2f %5.2f %5.2f\n",temp_bme,pres_bme,hum_bme);
    fprintf(pFile, "SENSOR %d-%d-%d %d:%d, %5.2f, %5.2f, %0.2lf, %0.2lf, %0.2lf\n",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,temp_hdc,hum_hdc,temp, press, hum);

    printf("SENSOR %d-%d-%d %d:%d, %5.2f, %5.2f, %0.2lf, %0.2lf, %0.2lf\n",tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,temp_hdc,hum_hdc,temp, press, hum);
 
    fclose(pFile); 


  */

  //  delay(10000);
  //}
  return 0 ;
}
