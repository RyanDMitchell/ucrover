/* ----------------------------------------------------------------------------
	File: filer.hpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	26/08/2015
   
	File Description: files data
				
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef FILER_HPP
#define FILER_HPP

#include <vector>		//std::vector
#include <string>		//std::string
#include <stdio.h>		//printf
#include <pthread.h> 	//multithreading
#include "stdint.h"		//Standard int types
#include <sys/time.h>	//time of day


/* --project includes-- */
#include "util.h"
#include "yei.hpp"
#include "yei_heading.hpp"
#include "lidar.hpp"
#include "lidar_lite.hpp"
#include "encoder.hpp"

class Filer{
	private:
		//selection vars
		bool useYei1;
		bool useYei2;
		bool useLidar;
		bool useLaser;
		bool useEncoder;
	
		//sensor objects
		Yei *yei1;
		Yei_heading *yei2;
		Lidar *lidar;
		LidarLite *laser;
		Encoder *encoder;

		//Timing and stats
		uint32_t startTime;
		uint32_t laserMeasurements;
		uint32_t lidarMeasurements;
		uint32_t IMU1Measurements;
		uint32_t IMU2Measurements;
		uint32_t encoderMeasurements;
		
		// thread functions
		pthread_t fileThread;
		void *file(void);
		static void* callFile(void* arg) {
			return ((Filer*)arg)->file(); 
		}
		
		//thread safe stream stopping/starting
		pthread_mutex_t stopMutex;
		int stopIssued;
		int getStop(void);
		void setStop(int val);
		
		//write to file method
		void writeDataToFile(std::string filePath, std::vector<std::string> data);

	public:
		Filer(std::string yei1Port, std::string yei2Port, std::string lidarPort, std::string encoderPort);
		~Filer();
		void startFiling();
		void stopFiling();
};

#endif //end Filer_HPP
