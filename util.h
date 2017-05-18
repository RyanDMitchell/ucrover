/* ----------------------------------------------------------------------------
	File: util.h

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: 	This file contains declarations for some useful utility 
						functions used by other files in this project.
	
	Notes: 	-Required dependency for the majority of the files in this project.

	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef UTIL_H
#define UTIL_H

#include <vector>		//std::vector
#include "stdint.h"	//Standard int types

//defines for extracting the high and low bytes
#define HI(x)  ((x) >> 8)
#define LO(x)  ((x) & 0xFF)

uint32_t GetTickCount();
void appendInt(uint32_t value, std::vector<uint8_t> &data);
void appendFloat(uint8_t bytes[4], std::vector<float> &data);
void appendFloatAsBytes(float value, std::vector<uint8_t> &data);
uint32_t bytesToInt(uint8_t bytes[4]);


#endif //end UTIL_H
