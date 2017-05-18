/* ----------------------------------------------------------------------------
	File: util.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: 	This file contains some useful utility functions used 
						by other files in this project.
	
	Notes: 	-Required dependency for the majority of the files in this project.

	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/

 /* --system includes --*/
#include <stdio.h>		//NULL
#include <sys/time.h>	//gettimeofday
#include <time.h>
#include "stdint.h"		//Standard int types
#include <string.h>		//memcpy
#include <vector>		//std::vector
#include <algorithm>    //std::reverse
#include <math.h>

 /* --project includes --*/
#include "util.h"

/* ----------------------------------------------------------------------------
	Function: uint32_t GetTickCount()
	
	Description: Returns the current time in milliseconds since the device 
				 was powered on.
   
	Parameters: None
		
	Returns: 	32-bit integer containing the time in milliseconds since the 
				device was powered on.
   ------------------------------------------------------------------------*/
uint32_t GetTickCount(){
    struct timeval tv;
    if( gettimeofday(&tv, NULL) != 0 ){
		return 0;
	}
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/* ----------------------------------------------------------------------------
	Function: void appendInt(uint32_t value, std::vector<uint8_t> &data)
	
	Description: Appends a 32 bit int as four bytes to a vector of bytes.
   
	Parameters:
		value: 	(Input) The 32-bit value to append to the vector of bytes.
		data:	(Output) The vector to append the four byte representation 
				of the 32-bit value.
		
	Returns: Nothing
	
	Notes: Stores the 32-bit value in Big-Endian format.
   ------------------------------------------------------------------------*/
void appendInt(uint32_t value, std::vector<uint8_t> &data){
	data.push_back( (value & 0xFF000000) >> 24 );
	data.push_back( (value & 0x00FF0000) >> 16 );
	data.push_back( (value & 0x0000FF00) >> 8 );
	data.push_back( (value & 0x000000FF));
}

/* ----------------------------------------------------------------------------
	Function: void appendFloat(uint8_t bytes[4], std::vector<float> &data)
	
	Description: Appends an array of 4 bytes representing of a 32-bit 
				 float to a vector of floats.
   
	Parameters:
		bytes: 	(Input) The array of 4 bytes representing the 32-bit float in 
				Big-Endian order
		data:	(Output) The vector to append the float to.
		
	Returns: Nothing	
	
	Notes: 	The 4 bytes array is read in Big-Endian format
   ------------------------------------------------------------------------*/
void appendFloat(uint8_t bytes[4], std::vector<float> &data){
 	float x;
	*((uint8_t *)&x+3) = bytes[0]; 
	*((uint8_t *)&x+2) = bytes[1]; 
	*((uint8_t *)&x+1) = bytes[2]; 
	*((uint8_t *)&x+0) = bytes[3];
	data.push_back(x); 
}

/* ----------------------------------------------------------------------------
	Function: uint32_t bytesToInt(uint8_t bytes[4])
	
	Description: Concatenates a 4 byte array into a 32bit integer
   
	Parameters:
		bytes: 	(Input) The array of 4 bytes representing the 32-bit integer in 
				Big-Endian order
		
	Returns: The 32-bit integer	
	
	Notes: 	The 4 bytes array is read as Big-Endian format
   ------------------------------------------------------------------------*/
uint32_t bytesToInt(uint8_t bytes[4]){
	return  (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

/* ----------------------------------------------------------------------------
	Function: void appendFloatAsBytes(float value, std::vector<uint8_t> &data)
	
	Description: Appends a 32-bit float as 4 bytes in big-endian format to a 
				 vector of bytes
   
	Parameters:
		value: 	(Input) The 32-bit float to append to the vector of bytes.
		data:   (Ouput) The Vector of bytes to append the float as bytes.
		
	Returns: Nothing
	
	Notes: 	The float is appended in Big-Endian format
   ------------------------------------------------------------------------*/
void appendFloatAsBytes(float value, std::vector<uint8_t> &data){
	const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
	/*Raspberry Pi represents data as Little-Endian so 
	  append in reverse order to make Big-Endian.      */
	data.push_back(bytes[3]);
	data.push_back(bytes[2]);
	data.push_back(bytes[1]);
	data.push_back(bytes[0]);
}