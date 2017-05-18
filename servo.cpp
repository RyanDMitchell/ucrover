/* ----------------------------------------------------------------------------
	File: servo.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	07/12/2015
   
	File Description: Communicates with an arduino over serial to set the position 
					  of a servo

	
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/

/*--system includes--*/
#include <stdio.h>		//printf
#include "stdint.h"		//Standard int types
#include <unistd.h> 	//usleep

/*--project includes--*/
#include "servo.hpp"

/* ----------------------------------------------------------------------------
	Function: Servo::Servo()
	
	Description: Constructor for the Servo class
   
	Parameters:
		port_name: 	(Input) A string containing the path of the device file used for 
					 the underlying serial connection. i.e. "/dev/ttyUSB0"
		baud:		(Input) The speed of the serial connection. Must be one of the 
					following set of bauds (1200, 2400, 4800, 9600, 19200,
					38400, 57600, 115200, 230400, 460800). If not from this 
					list the default of 115200 will be used.
	Returns: Nothing
   ------------------------------------------------------------------------*/
Servo::Servo(std::string port_name, unsigned int baud){

	//create serial object
	serial = new Serial(port_name, baud);

}

/* ----------------------------------------------------------------------------
	Function: Servo::~Servo()
	
	Description: Destructor for the Servo class
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Closes the serial
   ------------------------------------------------------------------------*/
Servo::~Servo(){
	printf("Servo: Closing serial...\n");
	serial->end();
	printf("Servo: Safetly closed\n");
}


/* ----------------------------------------------------------------------------
	Function: void Servo::setPos(int16_t pos)
	
	Description: Sets the servo to the specified position. Pos is taken as a 
				 pulse width in ms. Servos typically range from 1000-2000ms.
   
	Parameters:
		pos:	(Input) Pulse width to send to the servo
	
   ------------------------------------------------------------------------*/  
void Servo::setPos(uint16_t pos){
	//send 's' followed by 4 bytes representing the 4 digits of the value to set
	uint8_t byte0 = (pos / 1000) % 10;
	uint8_t byte1 = (pos / 100) % 10;
	uint8_t byte2 = (pos / 10) % 10;
	uint8_t byte3 = (pos % 10);
	
	uint8_t data[5] = {'s', byte0, byte1, byte2, byte3};
	int bytes = serial->send(data,5);
	if(bytes != 5){
		printf("Servo error: Error sending the position over serial ( Only %d bytes sent )\n",bytes);
		printf(" -- Tried to send: %d %d %d %d\n",byte0,byte1,byte2,byte3);
	}
}