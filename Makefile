#
# Makefile:
#	wiringPi - A "wiring" library for the Raspberry Pi
#	https://projects.drogon.net/wiring-pi
#
#	Copyright (c) 2012-2015 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#	Wiring Compatable library for the Raspberry Pi
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################

ifneq ($V,1)
Q ?= @
endif

DEBUG	= -g -O0
#DEBUG	= -O3
INCLUDE	= -I/usr/local/include -I ./
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lpthread -lm -lcrypt -lrt

# Should not alter anything below this line
###############################################################################

SRC	=	main.c bme280.c I2C.c MCP23S17.c SPI_daisy.c HDC2080.c 

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:
	$Q cat README.TXT
	$Q echo "    $(BINS)" | fmt
	$Q echo ""

really-all:	$(BINS)

main:	main.o bme280.o MCP23S17.o  SPI_daisy.o HDC2080.o   mcp3426.o I2C.o
	$Q echo [link]
	$Q $(CC) -o $@ main.o MCP23S17.o  mcp3426.o bme280.o I2C.o SPI_daisy.o HDC2080.o $(LDFLAGS) $(LDLIBS)



.c.oa:
	$Q echo [CC] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@

clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)

depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
