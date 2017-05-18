/* ----------------------------------------------------------------------------
	File: lidar_lite.hpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	26/08/2015
   
	File Description: This file dclares a threaded class to interface to a 
					  LidarLite device over a I2C connection. 
	
	Notes: 	-Roughly based on LidarLite code found at 
				https://github.com/answer17/lidarLite but rewritten to avoid 
				using the oversimplified WiringPi I2C functions. Uses 3rd party
				I2Cdev library code for I2C interfacing instead.
			-Uses pthreads for threading polling of the LidarLite as a 
				read request takes ~20ms.
				
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef LIDAR_LITE_HPP
#define LIDAR_LITE_HPP

#include <stdio.h>		//printf
#include "stdint.h"		//Standard int types
#include <pthread.h> 	//multithreading
#include <queue>		//STL queue 
#include <time.h>

#define LIDAR_LITE_ADRS 0x62
#define MEASURE_VAL 0x04
#define MEASURE_VAL_FAST 0x03

#define MODE_CONTROL_REG 0x04
#define MEASURE_REG 0x00
#define STATUS_REG  0x47
#define DISTANCE_REG_HI 0x0f
#define DISTANCE_REG_LO 0x10  
#define VERSION_REG 0x41

//mode control masks
#define PREAMP_OFF 0x80
#define DISABLE_REF_FILTER 0x08
#define INHIBIT_REF 0x02

// Status Bits
#define STAT_BUSY               0x01
#define STAT_REF_OVER           0x02
#define STAT_SIG_OVER           0x04
#define STAT_PIN                0x08
#define STAT_SECOND_PEAK        0x10
#define STAT_TIME               0x20
#define STAT_INVALID            0x40
#define STAT_EYE                0x80

class LidarLite{
	private:
	
		//I2C access
		pthread_mutex_t i2cMutex;
		uint16_t i2c_device;

		//I2C read/write
		bool writeByte(uint8_t regAddr, uint8_t value);
		int8_t readByte(uint8_t regAddr, uint8_t &data);
		
		//data access
		pthread_mutex_t dataMutex;
		std::queue<uint32_t> time_queue;
		std::queue<uint16_t> dist_queue;
		void setData(uint32_t time, uint16_t dist);
		
		// thread functions
		pthread_t pollThread;
		void *poll(void);
		static void* callPoll(void* arg) {
			return ((LidarLite*)arg)->poll(); 
		}
		
		//thread safe stopping 
		pthread_mutex_t sendStopMutex;
		int sendStopIssued;
		int getSendStop(void);
		void setSendStop(int val);
		
		//time stuff
		uint32_t calctimeus(struct timespec *start, struct timespec *now);
	
	public:
		LidarLite();
		~LidarLite();
		void start();
		void stop();
		bool getData(uint32_t &timeStamp, uint16_t &distance);
};
	
#endif //LIDAR_LITE_HPP
