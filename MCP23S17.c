/*
  MCP23S17.cpp  Version 0.1
  Microchip MCP23S17 SPI I/O Expander Class for Arduino
  Created by Cort Buffington & Keith Neufeld
  March, 2011

  Features Implemented (by word and bit):
    I/O Direction
    Pull-up on/off
    Input inversion
    Output write
    Input read

  Interrupt features are not implemented in this version
  byte based (portA, portB) functions are not implemented in this version

  NOTE:  Addresses below are only valid when IOCON.BANK=0 (register addressing mode)
         This means one of the control register values can change register addresses!
         The default values is 0, so that's how we're using it.

         All registers except ICON (0xA and 0xB) are paired as A/B for each 8-bit GPIO port.
         Comments identify the port's name, and notes on how it is used.

         *THIS CLASS ENABLES THE ADDRESS PINS ON ALL CHIPS ON THE BUS WHEN THE FIRST CHIP OBJECT IS INSTANTIATED!

  USAGE: All Read/Write functions except wordWrite are implemented in two different ways.
         Individual pin values are set by referencing "pin #" and On/Off, Input/Output or High/Low where
         portA represents pins 0-7 and portB 8-15. So to set the most significant bit of portB, set pin # 15.
         To Read/Write the values for the entire chip at once, a word mode is supported buy passing a
         single argument to the function as 0x(portB)(portA). I/O mode Output is represented by 0.
         The wordWrite function was to be used internally, but was made public for advanced users to have
         direct and more efficient control by writing a value to a specific register pair.
*/

#include "MCP23S17.h"            // Header files for this class
#include <linux/spi/spidev.h>

// Defines to keep logical information symbolic go here

#define    HIGH          (1)
#define    LOW           (0)
#define    ON            (1)
#define    OFF           (0)

// Here we have things for the SPI bus configuration

// Control byte and configuration register information - Control Byte: "0100 A2 A1 A0 R/W" -- W=0

#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3

// Constructor to instantiate an instance of MCP to a specific chip (address)


static const char * spidev[2][2] = {
    {"/dev/spidev0.0", "/dev/spidev0.1"},
    {"/dev/spidev1.0", "/dev/spidev1.1"},
};


static const uint8_t spi_mode = 0;
static const uint8_t spi_bpw = 8; // bits per word
static const uint32_t spi_speed = 5000000; // 10MHz
static const uint16_t spi_delay = 0;

void MCP_setup(MCP *mcp, uint8_t bus, uint8_t chip_select, uint8_t address) {

	mcp->_address     = address;

	int fd;
    // open
    if ((fd = open(spidev[bus][chip_select], O_RDWR)) < 0) {
        fprintf(stderr,
                "mcp23s17_open: ERROR Could not open SPI device (%s).\n",
                spidev[bus][chip_select]);
    }

    // initialise
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        fprintf(stderr, "mcp23s17_open: ERROR Could not set SPI mode.\n");
	close(fd);
    }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw) < 0) {
        fprintf(stderr,
                "mcp23s17_open: ERROR Could not set SPI bits per word.\n");
	close(fd);

    }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
        fprintf(stderr, "mcp23s17_open: ERROR Could not set SPI speed.\n");
	close(fd);

    }

    mcp->_spifd = fd;

MCP_byteWrite(mcp, IOCON, IOCON_INIT);
//        MCP_byteWrite(mcp, IOCON, 0);

 printf("MCPINI=%x %x %x\n",IOCON_INIT ,MCP_byteRead(mcp,IOCON),MCP_byteRead(mcp,OLATA));

//exit(1);
 mcp->_outputACache = MCP_byteRead(mcp, OLATA) ;
 mcp->_outputBCache = MCP_byteRead(mcp, OLATB) ;


};



void MCP_byteWrite(MCP *mcp, uint8_t reg, uint8_t value) {      // Accept the register and byte


  uint8_t tx_buf[3];
  tx_buf[0] = OPCODEW | (mcp->_address << 1);
  tx_buf[1] = reg;
  tx_buf[2] = value;

  uint8_t rx_buf[sizeof tx_buf];


  struct spi_ioc_transfer spi;
  memset (&spi, 0, sizeof(spi));
  spi.tx_buf = (unsigned long) tx_buf;
  spi.rx_buf = (unsigned long) rx_buf;
  spi.len = sizeof tx_buf;
  spi.delay_usecs = spi_delay;
  spi.speed_hz = spi_speed;
  spi.bits_per_word = spi_bpw;

  // do the SPI transaction
  if ((ioctl(mcp->_spifd, SPI_IOC_MESSAGE(1), &spi) < 0)) {
    fprintf(stderr,
	    "mcp23s17_read_reg: There was an error during the SPI "
	    "transaction.\n");

  }

}


// MODE SETTING FUNCTIONS - BY PIN 

void MCP_pinMode(MCP *mcp, uint8_t pin, uint8_t mode) {  // Accept the pin # and I/O mode

  uint8_t mask, old, reg ;

  if (pin > 15 ) fprintf(stderr, "Pin out of range on MCP23S17\n");
  
  if (pin < 8)          // Bank A
    reg  = IODIRA ;
  else
  {
    reg  = IODIRB ;
    pin &= 0x07 ;
  }

  mask = 1 << pin ;
  old  = MCP_byteRead(mcp, reg) ;

  if (mode == MCP_OUTPUT)
    old &= (~mask) ;
  else
    old |=   mask ;
  

  MCP_byteWrite(mcp, reg, old);                // Call the generic word writer with start register and the mode cache
}

// THE FOLLOWING WRITE FUNCTIONS ARE NEARLY IDENTICAL TO THE FIRST AND ARE NOT INDIVIDUALLY COMMENTED

// WEAK PULL-UP SETTING FUNCTIONS - BY WORD AND BY PIN

void MCP_pullupMode(MCP *mcp, uint8_t pin, uint8_t mode) {

  uint8_t mask, old, reg ;


  if (pin < 8)          // Bank A
    reg  = GPPUA ;
  else
  {
    reg  = GPPUB ;
    pin &= 0x07 ;
  }

  mask = 1 << pin ;
  old  = MCP_byteRead(mcp, reg) ;


  if (mode == PUD_UP)
    old |=   mask ;
  else
    old &= (~mask) ;
  MCP_byteWrite(mcp, reg, old);                // Call the generic word writer with start register and the mode cache


}



// WRITE FUNCTIONS 

void MCP_pinWrite(MCP *mcp, uint8_t pin, uint8_t value) {

  uint8_t bit;
  uint8_t old;
  bit = 1 << (pin & 7) ;

  if (pin < 8)                  // Bank A
  {
    old = mcp->_outputACache ;

    if (value == LOW)
      old &= (~bit) ;
    else
      old |=   bit ;

    MCP_byteWrite(mcp, GPIOA, old);                // Call the generic word writer with start register and the mode cache
    mcp->_outputACache = old ;
  }
  else                          // Bank B
  {
    old = mcp->_outputBCache ;

    if (value == LOW)
      old &= (~bit) ;
    else
      old |=   bit ;

    MCP_byteWrite(mcp, GPIOB, old);                // Call the generic word writer with start register and the mode cache
    mcp->_outputBCache = old ;
  }
}




uint8_t MCP_byteRead(MCP *mcp, uint8_t reg) {

  uint8_t tx_buf[3];
  tx_buf[0] = OPCODER | (mcp->_address << 1);
  tx_buf[1] = reg;
  tx_buf[2] = 0;

  uint8_t rx_buf[sizeof tx_buf];


  struct spi_ioc_transfer spi;
  memset (&spi, 0, sizeof(spi));
  spi.tx_buf = (unsigned long) tx_buf;
  spi.rx_buf = (unsigned long) rx_buf;
  spi.len = sizeof tx_buf;
  spi.delay_usecs = spi_delay;
  spi.speed_hz = spi_speed;
  spi.bits_per_word = spi_bpw;

  // do the SPI transaction
  if ((ioctl(mcp->_spifd, SPI_IOC_MESSAGE(1), &spi) < 0)) {
    fprintf(stderr,
	    "mcp23s17_read_reg: There was a error during the SPI "
	    "transaction.\n");
    return -1;
  }


	
  return rx_buf[2];


}

uint8_t MCP_pinRead(MCP *mcp, uint8_t pin) {              // Return a single bit value, supply the necessary bit (1-16)


  uint8_t mask, value, gpio ;

  if (pin < 8)          // Bank A
    gpio  = GPIOA ;
  else
  {
    gpio  = GPIOB ;
    pin  &= 0x07 ;
  }

  mask  = 1 << pin ;
  value = MCP_byteRead (mcp, gpio) ;

  if ((value & mask) == 0)
    return LOW ;
  else 
    return HIGH ;


}
