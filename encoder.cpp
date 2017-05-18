/* ----------------------------------------------------------------------------
	File: encoder.cpp

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

/*--system includes--*/
#include <stdio.h>		//printf
#include "stdint.h"		//Standard int types
#include <unistd.h> 	//usleep
#include <cerrno>		//error handling
#include <cstring>		//error strings
#include <pthread.h> 	//multithreading
#include <wiringPiSPI.h>//SPI Comms

/*--project includes--*/
#include "encoder.hpp"
#include "util.h"

/* ----------------------------------------------------------------------------
	Function: Encoder::Encoder()
	
	Description: Constructor for the Encoder class
   
	Parameters:
		port_name: 	(Input) A string containing the path of the device file used for 
					 the underlying serial connection. i.e. "/dev/ttyUSB0"
		baud:		(Input) The speed of the serial connection. Must be one of the 
					following set of bauds (1200, 2400, 4800, 9600, 19200,
					38400, 57600, 115200, 230400, 460800). If not from this 
					list the default of 115200 will be used.
	Returns: Nothing
	
	Notes: 	Initialise the required threading variables. Automatically 
			starts the device polling thread.
   ------------------------------------------------------------------------*/
Encoder::Encoder(std::string port_name, unsigned int baud){
	//vars
	stopIssued = 0;
	
	//create serial object
	serial = new Serial(port_name, baud);
	
	//Initialize the bloody mutexes
	pthread_mutex_init(&dataMutex,NULL);
	pthread_mutex_init(&stopMutex,NULL);
	
	//create the thread
	int result = pthread_create(&pollThread, 0, &Encoder::callPoll, this);
	if (result != 0){
		printf("Problem creating Encoder send thread. Code: %i\n",result);
		pthread_detach(pollThread);
	}
}
 
/* ----------------------------------------------------------------------------
	Function: Encoder::~Encoder()
	
	Description: Destructor for the Encoder class
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Sends the stop command to the thread and waits for it to end.
   ------------------------------------------------------------------------*/
Encoder::~Encoder(){
	setStop(1);
	pthread_join(pollThread, NULL);
	printf("Encoder: Closing serial...\n");
	serial->end();
	
	pthread_mutex_destroy(&stopMutex);
	pthread_mutex_destroy(&dataMutex);
	printf("Encoder: Safetly closed\n");
}

/* ----------------------------------------------------------------------------
	Function: void  Encoder::start()
	
	Description: Starts the device polling thread
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Does not check if a thread is already running, use carefully.
   ------------------------------------------------------------------------*/
void  Encoder::start(){
	//reset the stop command
	setStop(0);
	//create the thread
	int result = pthread_create(&pollThread, 0, &Encoder::callPoll, this);
	if (result != 0){
		printf("Problem creating Encoder send thread. Code: %i\n",result);
		pthread_detach(pollThread);
	}
}

/* ----------------------------------------------------------------------------
	Function: void  Encoder::stop()
	
	Description: Stops the device polling thread
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Does not check if a thread is already stopped, use carefully.
   ------------------------------------------------------------------------*/
void  Encoder::stop(){
	//set the stop flag to end the thread
	if(getStop() == 0){
		setStop(1);
		pthread_join(pollThread, NULL);
	}
}

/* ----------------------------------------------------------------------------
	Function: void Encoder::getData(uint32_t &timeStamp, uint32_t &countL, uint32_t &countR)
	
	Description: Returns the most recently obtained encoder counts 
				 from the Encoder device
   
	Parameters:
		timeStamp:	(Output): The timestamp of the distance measurement
		countL:		(Output): The count of the left encoder.
		countR:		(Output): The count of the right encoder.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/
bool Encoder::getData(uint32_t &timeStamp, int32_t &countL, int32_t &countR){
	bool gotData = false;
	//lock 
	pthread_mutex_lock(&dataMutex);
	//check if queues have data
	if(!time_queue.empty()){
		//pop oldest data from buffer queue
		timeStamp = time_queue.front();
		countL = L_queue.front();
		countR = R_queue.front();
		time_queue.pop();
		L_queue.pop();
		R_queue.pop();
		gotData = true;
	}else{
		gotData = false;
	}
	//unlock
	pthread_mutex_unlock(&dataMutex);
	return gotData;
}

/* ----------------------------------------------------------------------------
	Function: void Encoder::setData(uint32_t countL, uint32_t countR)
	
	Description: Private class method to set the shared data variables 
				 in a thread-safe way.
   
	Parameters: 
		countL:	(Input) Value to set the current left encoder count to.
		countR:	(Input) Value to set the current Right encoder count to.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/
void Encoder::setData(uint32_t time, int32_t countL, int32_t countR){
	// lock/unlock mutex for thread safety
	pthread_mutex_lock(&dataMutex);
	time_queue.push(time);
	L_queue.push(countL);
	R_queue.push(countR);
	pthread_mutex_unlock(&dataMutex);
}
	
/* ----------------------------------------------------------------------------
	Function: void* Encoder::poll(void)
	
	Description: Private threaded function used to poll the Encoder device. 
				 Updates the left and right encoder counts.
   
	Parameters: None
	Returns: Nothing
	
	Notes: As this function is a pointer to a c++ class method, when using 
		   pthread_create it must be called using the static callPoll() method
		   with "this" passed in as a parameter. 
		   i.e. pthread_create(&pollThread, 0, &Encoder::callPoll, this)
		   callPoll is defined in "lidar_lite.hpp"
   ------------------------------------------------------------------------*/
void* Encoder::poll(void){
	printf("Encoder thread: Started\n");
	bool isError = false;
	//The most recently aquired measurements
	int32_t currentCountL = 0;
	int32_t currentCountR = 0;
	int32_t newCountL = 0;
	int32_t newCountR = 0;
	
	//flush any bytes leftover on the serial
	uint8_t dummy;
	while(serial->available()){
		serial->get(&dummy, 1);
		printf("Encoder: flushing a byte from the serial buffer\n");
	}
	
	//setup microsecond time
	struct timespec time_start;
	clock_gettime(CLOCK_REALTIME , &time_start);
	
	while(getStop() == 0 && !isError){
		//get the counts from the arduino
		if(getCounts(newCountL, newCountR)){
			//only update the data when the counts change
			//if((newCountL != currentCountL) || (newCountR != currentCountR)){
				//get microsecond time 
				struct timespec time_now;
				clock_gettime(CLOCK_REALTIME , &time_now);
				uint32_t us = calctimeus(&time_start, &time_now);
				setData(us, newCountL, newCountR);
				currentCountL = newCountL;
				currentCountR = newCountR;
			//}
		}else{
			printf("Encoder: Invalid read ignored\n");
		}
		//usleep(50);
	}//end while(getStop() == 0)
	
	printf("Encoder thread: Stopped\n");
	pthread_exit(NULL);
}

/* ----------------------------------------------------------------------------
	Function: void Encoder::getCounts(int32_t &countL, int32_t &countR)
	
	Description: Read the encoder counts from the serial connection
   
	Parameters:
		countL:	(Output) variable to write the left count value to.
		countR:	(Output) variable to write the right count value to.
		
	Returns: true if successful 
   ------------------------------------------------------------------------*/  
bool Encoder::getCounts(int32_t &countL, int32_t &countR){
	bool ret = true;

	//send 'r' to tell the arduino to send
	uint8_t countCmd = 0x72;
	int bytes = serial->send(&countCmd,1);
	if(bytes != 1){
		printf("Encoder error: Error sending the request byte over serial\n");
	}else{
		//read 8bytes from the serial
		//data should be in the form [countL0, countL1, countL2, countL3, countR0, countR1, countR2, countR3]
		uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		int bytesReceived = serial->get(data, 8);
		if(bytesReceived != 8){
			printf("Encoder error: Not enough bytes received from the serial connection (%i bytes).\n", bytesReceived);
			ret = false;
		}else{
			//left counter data first, right second
			countL = bytesToInt(&data[0]);
			countR = bytesToInt(&data[4]);
		}
	}
	return ret;
}

 
/* ----------------------------------------------------------------------------
	Function: int Encoder::getStop(void)
	
	Description: Private method to safely get the value of the stop command 
   
	Parameters: None
	Returns: The current value of the stop command
	
	Notes: Anything other than zero stops the polling thread
   ------------------------------------------------------------------------*/  
int Encoder::getStop(void){
	int ret = 0;
	//Lock up the shared variable
	pthread_mutex_lock(&stopMutex);
	ret = stopIssued;
	pthread_mutex_unlock(&stopMutex);
	return ret;
}

/* ----------------------------------------------------------------------------
	Function: void Encoder::setStop(int val) 
	
	Description: Private method to send a command to safely stop the polling thread
   
	Parameters: 
		val: (input) The value to write to the stop command. 
					 Anything other than 0 stops the thread.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/  
void Encoder::setStop(int val) {
	//Lock up the shared variable
	pthread_mutex_lock(&stopMutex);
	stopIssued = val;
	pthread_mutex_unlock(&stopMutex);
}

/* ----------------------------------------------------------------------------
	Function: uint32_t Encoder::calctimeus(struct timespec *start, struct timespec *now)
	
	Description: Helper function with takes a start timespec and a current 
				 timespec and calcualtes the difference between them in 
				 microseconds

   ------------------------------------------------------------------------*/  
uint32_t Encoder::calctimeus(struct timespec *start, struct timespec *now){
	struct timespec result;
    if ((now->tv_nsec - start->tv_nsec) < 0) {
        result.tv_sec = now->tv_sec - start->tv_sec - 1;
        result.tv_nsec = now->tv_nsec - start->tv_nsec + 1e9;
    } else {
        result.tv_sec = now->tv_sec - start->tv_sec;
        result.tv_nsec = now->tv_nsec - start->tv_nsec;
    }
	
	uint32_t us = (uint32_t)(result.tv_sec*1e6 + result.tv_nsec/1e3);
    return us;
}
