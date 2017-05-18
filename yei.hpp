/* ----------------------------------------------------------------------------
	File: yei.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: This file declares a class to interface to a 3-space 
					  IMU device over a serial connection. 
			
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef YEI_HPP
#define YEI_HPP

#include <vector>	//std::vector
#include <string>	//std::string
#include <stdio.h>	//printf
#include "stdint.h"	//Standard int types
#include <string.h>	//memcpy
#include <queue>	//STL queue 

#include "serial.hpp"
#include "util.h"

class Yei{
	private:
		//serial object
		Serial *serial;
		uint32_t updateRate;
		uint8_t checksum(std::vector<uint8_t> data);
		
		// thread functions
		pthread_t yeiThread;
		void *update(void);
		static void* callUpdate(void* arg) {
			return ((Yei*)arg)->update(); 
		}
		
		//thread safe stopping 
		pthread_mutex_t stopMutex;
		int stopIssued;
		int getStop(void);
		void setStop(int val);
		
		//thread safe data access
		pthread_mutex_t dataMutex;
		std::queue<uint32_t> time_queue;
		std::queue< std::vector<float> > data_queue;
		void setData(uint32_t timeStamp, std::vector<float> data);

		
	public:
		Yei(std::string port_name, unsigned int baud);
		~Yei();
		void start(uint32_t updateRate, uint32_t streamDuration, uint32_t initDelay);
		void stop();
		bool getData(uint32_t &timeStamp, std::vector<float> &data);
		void printSerialNumber();
		int getQueueSize();

};



#endif //end YEI_HPP
