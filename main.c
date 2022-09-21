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
#include "mcp3426.h"
#include <signal.h>


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
I2C i2c_cramps[24];

#define MAX_LINE_LENGTH 24


mcp3426_t adc[48];
int enabledSlots[MAX_LINE_LENGTH];



void intHandler(int dummy) {
    keepRunning = 0;
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

void initialization(){

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


    char *path;
    char line[MAX_LINE_LENGTH] = {0};

    // read enabled channels from input.txt
    path = "input.txt";

    int slotCounter = 0;
    FILE *file = fopen(path, "r");
    for (int i = 0; i < MAX_LINE_LENGTH; i++) {
        enabledSlots[i] = -1;
    }
    while(fgets(line, MAX_LINE_LENGTH, file)){
        enabledSlots[slotCounter] = atoi(line);
        slotCounter++;
    }
    fclose(file);


    //initialize cramps
    for (int i = 0; i < MAX_LINE_LENGTH; i++) {


        if (enabledSlots[i] >= 0) {
            printf("Enabling slot %d\n", enabledSlots[i]);
            if (enabledSlots[i]>=0 && enabledSlots[i] <14){
	      I2C_setup(&i2c_cramps[i],&crampMCP[MCPHV0], 1, &crampMCP[MCPHV0], enabledSlots[i]+2); //SCL SDA
	      _mcp3426_init(&adc[enabledSlots[i]], &i2c_cramps[i]);
            }
            if (enabledSlots[i]>=14 && enabledSlots[i] <24){
	      I2C_setup(&i2c_cramps[i],&crampMCP[MCPHV0], 1 , &crampMCP[MCPHV1], enabledSlots[i]-14);
	      _mcp3426_init(&adc[enabledSlots[i]], &i2c_cramps[i]);

	      //  _mcp3426_init(&adc[enabledSlots[i]], MCPPINBASE+16*MCPHV1+(enabledSlots[i]-14),MCPPINBASE+16*MCPHV0+1); 
            }
            if (enabledSlots[i]>=24 && enabledSlots[i] <38){
	      I2C_setup(&i2c_cramps[i],&crampMCP[MCPHV2], 1, &crampMCP[MCPHV2], enabledSlots[i]-24+2);
	      _mcp3426_init(&adc[enabledSlots[i]], &i2c_cramps[i]);

	    // _mcp3426_init(&adc[enabledSlots[i]], MCPPINBASE+16*MCPHV2+(enabledSlots[i]-24+2),MCPPINBASE+16*MCPHV2+1);       
            }
            if (enabledSlots[i]>=38 && enabledSlots[i] <48){
	      I2C_setup(&i2c_cramps[i],&crampMCP[MCPHV2], 1, &crampMCP[MCPHV3], enabledSlots[i]-38);
	      _mcp3426_init(&adc[enabledSlots[i]], &i2c_cramps[i]);

	      //_mcp3426_init(&adc[enabledSlots[i]], MCPPINBASE+16*MCPHV3+(enabledSlots[i]-38),MCPPINBASE+16*MCPHV2+1); 
            }
        }
    }



}



int main(int argc, char *argv[])
{
  FILE *pFile;
  pFile=fopen("htp_2.txt", "a+");
  initialization();


  char * outputFilename = "currvstime.csv";
  FILE *fpt;
  fpt = fopen(outputFilename, "w+"); //output file
   
  
  //  int channel = atoi(argv[1]);


  int n = 0;
  time_t tic = time(NULL);
  time_t toc = time(NULL); //define clock ticks

  float current1 = 0;
  float current2 = 0;
  float ave1 = 0;
  float ave2 = 0;

  clock_t t;
  t = clock();


  while (keepRunning){
          time_t t = time(NULL);
        struct tm tm = *localtime(&t);
        fprintf(fpt, "%d-%d-%d %d:%d:%d,", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

  for (int i = 0; i < MAX_LINE_LENGTH; i++) {

  
    if (enabledSlots[i] >= 0) {
      current1 = 0;
      current2 = 0;

        int tmp = 0;
	tmp =  _mcp3426_setconfig(&adc[enabledSlots[i]],0,2,0,0);
	current1 = 1e2*_mcp3426_read(&adc[enabledSlots[i]]);
        /* if (i%50==0){ */
	//        printf( "---z%d (%.0f)---\n", enabledSlots[i],  ((double)(toc - tic)));
	//        printf( "current 1 (nA) = %4.3f\n",current1);
        /* } */
        tmp = 0;
	tmp = _mcp3426_setconfig(&adc[enabledSlots[i]],0,2,0,1);
	current2 = 1e2*_mcp3426_read(&adc[enabledSlots[i]]);

        /* if (i%50==0){ */
	//        printf( "current 2 (nA) = %4.3f\n",current2);
	//        printf( "\n" );
        /* } */
        ave1 = ave1+current1;
        ave2 = ave2+current2;
        n = n + 1;

	fprintf(fpt, "%.3f,%.3f,", current1, current2);

      }
  }
  fprintf(fpt,"\n");
  }
  toc = time(NULL);

  t = clock() - t;
  double time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
  printf( "--- (%4.2f)---\n",  time_taken);
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
