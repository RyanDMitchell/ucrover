/* ----------------------------------------------------------------------------
	File: encoder.hpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	31/08/2015
   
	File Description: Communicates with an arduino over serial to get the counts from the 
					  encoders provided by the LS7366s.
	
	Notes: 	
	-Datasheets, and code examples found at(31/08/2015):
		http://www.superdroidrobots.com/shop/item.aspx/dual-ls7366r-quadrature-encoder-buffer/1523/
	-Arduino is programmed to send the left and right counts over 19200 8N1 
		serial whenever it receives a 0xFF byte. The count data is sent as 
		<Left 32bit int> <Right 32bit int> in Big endian form.
		
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/
#ifndef ENCODER_HPP
#define ENCODER_HPP

/* --project includes-- */
#include "serial.hpp"

/* --system includes-- */
#include <stdio.h>		//printf
#include "stdint.h"		//Standard int types
#include <pthread.h> 	//multithreading
#include <queue>

//The period of time in ms required before the vehicle is considered stationary
#define STATIONARY_THRESHOLD 1000

class Encoder{
	private:
		//serial object
		Serial *serial;
		
		//data variables
		std::queue<uint32_t>time_queue;
		std::queue<int32_t> L_queue; 
		std::queue<int32_t> R_queue;
		//mutexes to access the data
		pthread_mutex_t dataMutex;
		void setData(uint32_t time, int32_t countL, int32_t countR);

		// thread functions
		pthread_t pollThread;
		void *poll(void);
		static void* callPoll(void* arg) {
			return ((Encoder*)arg)->poll();
		}
		
		//thread safe stopping 
		pthread_mutex_t stopMutex;
		int stopIssued;
		int getStop(void);
		void setStop(int val);
		
		//time stuff
		uint32_t calctimeus(struct timespec *start, struct timespec *now);
		
		//grab counts function
		bool getCounts(int32_t &countL, int32_t &countR);
		
	public:
		Encoder(std::string port_name, unsigned int baud);
		~Encoder();
		void start();
		void stop();
		bool getData(uint32_t &timeStamp, int32_t &countL, int32_t &countR);
};

#endif //ENCODER_HPP
