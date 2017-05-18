/* ----------------------------------------------------------------------------
	File: yei.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: This file implements a class to interface to a 3-space 
					  IMU device over a serial connection. 
	
	Notes: 	-Code was written form this class using the 3-space user manual located at:
				http://www.yeitechnology.com/sites/default/files/YEI_TSS_Users_Manual_3.0_r1_4Nov2014.pdf
				downloaded on 25/08/2015.
			-Is non-threaded, so all functions are blocking.
			-Communication protocol for the 3-space sensor:
				-Binary format for a packet with N bytes-
						   1 | 0xF7 - Binary packet byte
						   2 | Command Byte
					3->(N-1) | Data Bytes
						   N | Checksum
					Data is returned in Big-Endian as either: 
						-Floating point 4 bytes single precision
						-Integer 
				-Checksum-
					Sum of all characters in the packet modulus 256 
					(except the first)
				
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/

   /* --system includes --*/
#include <stdio.h>		//printf
#include <vector>		//std::vector
#include <string>		//std::string
#include "stdint.h"		//Standard int types
#include <stdlib.h>		//RNG functions
#include <cerrno>		//error handling
#include <string.h>		//memcpy
#include <numeric>      //std::accumulate
#include <queue>		//STL queue 

/* --project includes --*/
#include "util.h"
#include "serial.hpp"
#include "yei.hpp"

/* ----------------------------------------------------------------------------
	Function: Yei::Yei(std::string port_name, unsigned int baud)
	
	Description: Constructor for the Yei Class. Attempts to open a serial 
				 connection on the specified port at the specified baud.
   
	Parameters:
		port_name: 	(Input) A string containing the path of the device file used for 
					the serial connection. i.e. "/dev/ttyUSB0"
		baud:		(Input) The speed of the serial connection. Must be one of the 
					following set of bauds (1200, 2400, 4800, 9600, 19200,
					38400, 57600, 115200, 230400, 460800). If not from this 
					list the default of 115200 will be used.
	Returns: Nothing
	
	Notes: -Uses the Serial class in serial.cpp, refer for more details about
				the serial comms.
   ------------------------------------------------------------------------*/
Yei::Yei(std::string port_name, unsigned int baud){
	updateRate = 0;
	stopIssued = 0;
	serial = new Serial(port_name, baud);
	
	//Initialize the bloody mutexes
	pthread_mutex_init(&dataMutex,NULL);
	pthread_mutex_init(&stopMutex,NULL);
}

/* ----------------------------------------------------------------------------
	Function: void Yei::~Yei()
	
	Description: Destructor for the Yei class. Closes the serial port.
   
	Parameters: None
	Returns: Nothing
   ------------------------------------------------------------------------*/
Yei::~Yei(){
	//stop the yei streaming (also ends the thread)
	stop();
	//end the serial
	printf("Yei: Closing serial connection...\n");
	serial->end();
	
	//Destroy the mutexes
	pthread_mutex_destroy(&stopMutex);
	pthread_mutex_destroy(&dataMutex);
	
	printf("Yei: Safetly closed\n");
}

/* ----------------------------------------------------------------------------
	Function: void Yei::printSerialNumber()
	
	Description: Prints the the serial number of the yei device to stdio
   
	Parameters: None
	Returns: Nothing
   ------------------------------------------------------------------------*/
void Yei::printSerialNumber(){
	/*Send the command:
	0xF7 - Binary bytes
	0xED - get serial number command
	0xED - Checksum (Sum of all bar first)*/
	uint8_t serialCommand[3] = {0xF7, 0xED, 0xED};
	serial->send(serialCommand,3);

	uint8_t rawData[4];
	int numBytes = serial->get(rawData,4);
	printf("Serial: ");
	for(int i = 0 ;i < numBytes ; i++){
		printf("%x ",rawData[i]);
	}
	printf("\n");
}

/* ----------------------------------------------------------------------------
	Function: void Yei::startStream(uint32_t updateRate, 
									uint32_t streamDuration, 
									uint32_t initDelay)
	
	Description: Communicates with the yei device to setup and start streaming 
				 IMU data. Data to be streamed is timestamped and corrected 
				 accelerometer and rate gyro.
   
	Parameters: 
		updateRate: 	(Input): Period of the streaming measurements in us. 
						0x00000000 is as fast as possible. 
		streamDuration:	(Input): The amount of time to stream for in us. 
						0xFFFFFFFF sets it to stream indefinitely.
		initDelay:		(Input) The time delay before the device starts streaming data. 
						0x00000000 is zero delay.
	Returns: Nothing
   ------------------------------------------------------------------------*/
void Yei::start(uint32_t updateRate, uint32_t streamDuration, uint32_t initDelay){
	//save the rate
	this->updateRate = updateRate;
	
	/*synchronize time*/
	std::vector<uint8_t> syncCommand;
	syncCommand.push_back(0xF7); //Binary packet byte
	syncCommand.push_back(0x5F); //set timestamp command
	//command data
	/*Data is a 4 byte value in microseconds*/
	uint16_t syncTime = GetTickCount()*1000;
	appendInt(syncTime,syncCommand);
	//Checksum byte
	syncCommand.push_back(checksum(syncCommand));
	//Send command
	serial->send(&syncCommand.front(),syncCommand.size());	
	
	/* Set up steaming slots */
	std::vector<uint8_t> streamCommand;
	streamCommand.push_back(0xF7); //Binary packet byte
	streamCommand.push_back(0x50); //Command byte
	//command data
	streamCommand.push_back(0x27); //get corrected accel
	streamCommand.push_back(0x26); //get corrected gyro 
	streamCommand.push_back(0x28); //get magnetometer
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(0xFF);
	streamCommand.push_back(checksum(streamCommand)); //Checksum byte
	//Send command
	serial->send(&streamCommand.front(),streamCommand.size());

	/* Set up stream timing */
	std::vector<uint8_t> timingCommand;
	//Binary packet byte
	timingCommand.push_back(0xF7);
	//Command byte
	timingCommand.push_back(0x52);
	//Command data
	appendInt(updateRate,timingCommand);
	appendInt(streamDuration,timingCommand);
	appendInt(initDelay,timingCommand);
	//Checksum byte
	timingCommand.push_back(checksum(timingCommand));
	//Send command
	serial->send(&timingCommand.front(),timingCommand.size());
	
	/* Setup response header for the timestamp */
	std::vector<uint8_t> resCommand;
	resCommand.push_back(0xF7); //Binary packet byte
	resCommand.push_back(0xDB); //Command byte
	//command data
	/*Data is a 4 byte bitmask to indicate the data to be returned in the header.
	Only the lowest 7 bits are used*/
	appendInt(0x00000002,resCommand); //flip the timestamp
	//Checksum byte
	resCommand.push_back(checksum(resCommand));
	//Send command
	serial->send(&resCommand.front(),resCommand.size());	
	
	/*start steaming data*/
	std::vector<uint8_t> startCommand;
	startCommand.push_back(0xF9);	//Binary packet with response header byte
	startCommand.push_back(0x55); 	//Command byte
	startCommand.push_back(0x55);   //Checksum byte
	//Send command
	serial->send(&startCommand.front(),startCommand.size());
	
	//read the response header back so it doesn't put the stream out of alignment
	//(Returns 5 bytes even through specified to return 4)
	uint8_t dummy[5];
	serial->get(dummy,5);
	
	//reset the stop command just in case
	setStop(0);
	//Start the update thread	
	int result = pthread_create(&yeiThread, 0, &Yei::callUpdate, this);
	if (result != 0){
		printf("Yei Error: Problem creating update thread. Code: %i\n",result);
		pthread_detach(yeiThread);
	}
}

/* ----------------------------------------------------------------------------
	Function: void Yei::getData(uint32_t &timeStamp, std::vector<float> &data)
	
	Description: Returns the most current set of data obtained from the Yei 
				 sensor in a threadsafe way
   
	Parameters:
	timeStamp:	(Output) The long to write the received yei timestamp to.	
		data:	(Output) The vector of floats where the IMU data is written.
		
	Returns: true if there was data
   ------------------------------------------------------------------------*/
bool Yei::getData(uint32_t &timeStamp, std::vector<float> &data){
	bool gotData = false;
	//lock 
	pthread_mutex_lock(&dataMutex);
	//check if queues have data
	if(!data_queue.empty()){
		//pop oldest data from buffer queue
		timeStamp = time_queue.front();
		data = data_queue.front();
		data_queue.pop();
		time_queue.pop();
		gotData = true;
	}else{
		gotData = false;
	}
	//unlock
	pthread_mutex_unlock(&dataMutex);
	return gotData;
}

/* ----------------------------------------------------------------------------
	Function: void Yei::setData(uint32_t timeStamp, std::vector<float> data)
	
	Description: sets the shared timestamp and data variables in a threadsafe way
   
   
	Parameters:
	timeStamp:	(Input) The long containing the timestamp.	
		data:	(Input) The vector of floats containing the IMU data.
		
	Returns: Nothing
   ------------------------------------------------------------------------*/
void Yei::setData(uint32_t timeStamp, std::vector<float> data){
	//lock and push new data onto buffer queue
	pthread_mutex_lock(&dataMutex);
	time_queue.push(timeStamp);
	data_queue.push(data);
	pthread_mutex_unlock(&dataMutex);
}

/* --------------------------------------------------------------------------
	Function: void* Streamer::stream(void)
	
	Description: Private threaded function that gets the currently steaming data 
				 from the yei device and saves it to the shared timeStamp and data
				 variables.
				 
	Parameters: None
	Returns: Nothing
	
	Notes: 
   ------------------------------------------------------------------------*/
void* Yei::update(void){
	printf("Yei thread: Started\n");
	uint32_t measurementCount = 0;
	uint32_t startTime = GetTickCount();
	/*Data comes as 41 byte blocks:
	1 random extra byte for some reason
	timestamp(uS) 4 byte int * 1
	Gyro 4 byte float * 3
	Accel 4 byte float * 3 
	compass 4 byte float * 3*/
	while(getStop() == 0){
		uint8_t rawData[41];
		std::vector<float> inData;
		inData.reserve(6);
		
		int bytesReceived = serial->get(rawData, 41);
		if(bytesReceived == 41){
			//*ignore the random unwanted first byte*
			uint32_t inTimeStamp = bytesToInt(&rawData[1]); //<- timestamp from device; 
			//uint32_t inTimeStamp = GetTickCount(); 		//<- use the RPi clock for timestamps
			for(int i = 5; i < bytesReceived; i = i + 4){
				//format into floating point and append to data vector
				appendFloat(&rawData[i],inData);
			}
			//thread safe update the data
			setData(inTimeStamp, inData);
			measurementCount++;
		}else{
			//error with serial port
			printf("Yei Thread Error: Error reading serial port. Exiting thread.\n");
			stop();
		} 
	}//end while(getStop() == 0)
	
	printf("Yei sensor thread: time: %d count: %d rate: %f\n", (GetTickCount() - startTime), measurementCount, 1.0 * measurementCount / (GetTickCount() - startTime) * 1000.0);

	printf("Yei thread: Stopped\n");
	pthread_exit(NULL);
}

/* ----------------------------------------------------------------------------
	Function: void Yei::stopStream()
	
	Description: Using binary comms mode, sends a command to the yei to stop 
				 it from steaming data.
   
	Parameters: None
	Returns: Nothing
   ------------------------------------------------------------------------*/
void Yei::stop(){
	if(getStop() == 0){
		//Send the stop streaming command to the yei
		std::vector<uint8_t> stopCommand;
		//Binary packet byte
		stopCommand.push_back(0xF7);
		//Command byte
		stopCommand.push_back(0x56);
		//Checksum byte
		stopCommand.push_back(checksum(stopCommand));
		//send it
		serial->send(&stopCommand.front(),stopCommand.size());
		
		//stop the thread
		setStop(1);
		pthread_join(yeiThread, NULL);
	}
}

/* ----------------------------------------------------------------------------
	Function: uint8_t checksum(std::vector<uint8_t> data)
	
	Description: Calculate the checksum for the 3-space binary packet
   
	Parameters: None
	Returns: Nothing
	
	Notes: -Checksum is equal to the sum of all characters in the 
				packet(bar the first) modulus 256
   ------------------------------------------------------------------------*/
uint8_t Yei::checksum(std::vector<uint8_t> data){
	uint8_t sum_of_elems = std::accumulate(data.begin(),data.end(),0);
	sum_of_elems = sum_of_elems - data[0];
	return sum_of_elems % 256;
}

/* --------------------------------------------------------------------------
	Function: int Yei::getStop(void)
	
	Description: Private method to safely get the value of the stop command 
   
	Parameters: None
	Returns: The current value of the stop command
	
	Notes: Anything other than zero stops the update thread 
   ------------------------------------------------------------------------*/  
int Yei::getStop(void){
	int ret = 0;

	pthread_mutex_lock(&stopMutex);
	ret = stopIssued;
	pthread_mutex_unlock(&stopMutex);
	return ret;
	}


/* --------------------------------------------------------------------------
	Function: void Yei::setStop(int val)
	
	Description: Private method to send a command to safely stop the 
				 update thread
   
	Parameters: 
		val: (input) The value to write to the stop command. 
					 Anything other than 0 stops the thread.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/  
void Yei::setStop(int val) {
	pthread_mutex_lock(&stopMutex);
	stopIssued = val;
	pthread_mutex_unlock(&stopMutex);
}

/* --------------------------------------------------------------------------
	Function: int Yei::getQueueSize()
	
	Description: Method to determine how large the buffer is
	Returns: size of the buffer
   ------------------------------------------------------------------------*/  
int Yei::getQueueSize() {
	//don't call too often
	int size = 0;
	pthread_mutex_lock(&dataMutex);
	size = time_queue.size();
	//safety check
	if(time_queue.size() != data_queue.size()){
		printf("Yei thread error: queue size mismatch!!\n");
	}
	pthread_mutex_unlock(&dataMutex);
	return size;
}

