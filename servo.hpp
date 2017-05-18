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
#ifndef SERVO_HPP
#define SERVO_HPP

/* --project includes-- */
#include "serial.hpp"

/* --system includes-- */
#include <stdio.h>		//printf
#include "stdint.h"		//Standard int types


class Servo{
	private:
		//serial object
		Serial *serial;
	
	public:
		Servo(std::string port_name, unsigned int baud);
		~Servo();
		void setPos(uint16_t pos);

};

#endif //SERVO_HPP
