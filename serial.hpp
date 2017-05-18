/* ----------------------------------------------------------------------------
	File: serial.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	24/08/2015
   
	File Description: This file declares a class which provides a serial 
					  connection with a specified port and baud using termios. 
					  The serial connection is statically set to 8N1 with CTSRTS 
					  enabled.
	
	Note: All functions are blocking.
	
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <string>
#include "stdint.h"	    //Standard int types
#include <poll.h>		//UART Polling

//serial read timeout in milliseconds
#define READTIMEOUT 500

class Serial{
	private:
	int uart0_filestream;
	
	public:
	Serial(std::string port_name, unsigned int baud);
	void end();
	int send(uint8_t *data, uint16_t numBytes);
	int get(uint8_t *data, uint16_t numBytes);
	int getline(uint8_t *data);
	bool available();
	
};

bool unitTestSerial();

#endif //end SERIAL
