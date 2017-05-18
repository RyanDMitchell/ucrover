/* ----------------------------------------------------------------------------
	File: lidar.hpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: This file declares a class which interfaces to a 
					  RPLidar device over a serial connection. 

	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef LIDAR_H
#define LIDAR_H

#include <stdio.h>
#include <stdlib.h>
#include <string>	//std::string
#include "stdint.h"		//Standard int types
#include "rplidar.h" 	//rplidar sdk
#include <vector>	//std::vector
#include <queue>
#include <time.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

class Lidar{
	private:
		
		RPlidarDriver *drv;
		std::string port_name;
		unsigned int baud;
		
		//shared variable declarations
		std::queue<float> queue_angle;
		std::queue<float> queue_distance;
		std::queue<uint8_t> queue_quality;
		std::queue<uint32_t> queue_time;
		
		//mutexes/functions to access the shared data
		pthread_mutex_t scanDataMutex;
		void setScanData(std::vector<uint32_t> time, std::vector<float> angle, std::vector<float> distance, std::vector<uint8_t> quality);

		// thread functions
		pthread_t lidarThread;
		void *pollLidar(void);
		static void* callPollLidar(void* arg) {
			return ((Lidar*)arg)->pollLidar(); 
		}
		
		//thread safe stopping 
		pthread_mutex_t stopMutex;
		int stopIssued;
		int getStop(void);
		void setStop(int val);
		
		//time stuff
		uint32_t calctimeus(struct timespec *start, struct timespec *now);
		
	public: 
		Lidar(std::string port_name, unsigned int baud);
		~Lidar();
		bool getScanData(uint32_t &timeStamp, float &angle, float &distance, uint8_t &quality);
		void start();
		void stop();
		int getQueueSize();
	
};

#endif //end LIDAR_H