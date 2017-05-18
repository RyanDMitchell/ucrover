/* ----------------------------------------------------------------------------
	File: lidar_lite.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: This file implements a threaded class to interface to a 
					  LidarLite device over a I2C connection. 
	
	Notes: 	-Roughly based on LidarLite code found at 
				https://github.com/answer17/lidarLite but rewritten to avoid 
				using the oversimplified WiringPi I2C functions. Uses 3rd party
				I2Cdev library code for I2C interfacing instead.
			-Set the RPI i2c speed to 400KHz by creating:
				options i2c_bcm2708 baudrate=400000
				in the file
				/etc/modprobe.d/custom.conf
				
				
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
#include <linux/i2c-dev.h> //I2C
#include <sys/ioctl.h>	//system read/writes
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>	//STL queue 
#include <time.h>

/*--3rd party includes--*/
#include "I2Cdev.h"		//I2C comms

/*--project includes--*/
#include "lidar_lite.hpp" 
#include "util.h"

/* ----------------------------------------------------------------------------
	Function: LidarLite::LidarLite()
	
	Description: Constructor for the LidarLite class
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Initialise the required threading variables. Automatically 
			starts the device polling thread.
   ------------------------------------------------------------------------*/
LidarLite::LidarLite(){
	sendStopIssued = 0;

	//Initialize the bloody mutexes
	pthread_mutex_init(&dataMutex,NULL);
	pthread_mutex_init(&sendStopMutex,NULL);
	pthread_mutex_init(&i2cMutex,NULL);
	
	//create the thread
	int result = pthread_create(&pollThread, 0, &LidarLite::callPoll, this);
	if (result != 0){
		printf("Problem creating LidarLite send thread. Code: %i\n",result);
		pthread_detach(pollThread);
	}
}

/* ----------------------------------------------------------------------------
	Function: LidarLite::~LidarLite()
	
	Description: Destructor for the LidarLite class
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Sends the stop command to the thread and waits for it to end.
   ------------------------------------------------------------------------*/
LidarLite::~LidarLite(){
	setSendStop(1);
	pthread_join(pollThread, NULL);
	
	pthread_mutex_destroy(&i2cMutex);
	pthread_mutex_destroy(&sendStopMutex);
	pthread_mutex_destroy(&dataMutex);
}

/* ----------------------------------------------------------------------------
	Function: void  LidarLite::start()
	
	Description: Starts the device polling thread
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Does not check if a thread is already running, use carefully.
   ------------------------------------------------------------------------*/
void  LidarLite::start(){
	//reset the stop command
	setSendStop(0);
	//create the thread
	int result = pthread_create(&pollThread, 0, &LidarLite::callPoll, this);
	if (result != 0){
		printf("Problem creating LidarLite send thread. Code: %i\n",result);
		pthread_detach(pollThread);
	}
}

/* ----------------------------------------------------------------------------
	Function: void  LidarLite::stop()
	
	Description: Stops the device polling thread
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Does not check if a thread is already stopped, use carefully.
   ------------------------------------------------------------------------*/
void  LidarLite::stop(){
	//set the stop flag to end the thread
	if(getSendStop() == 0){
		setSendStop(1);
		pthread_join(pollThread, NULL);
		printf("Laser: Safetly closed\n");
	}
}

/* ----------------------------------------------------------------------------
	Function: void LidarLite::getData(uint32_t &timeStamp, uint16_t &distance)
	
	Description: Returns the most recently obtained distance measurement 
				 from the LidarLite device
   
	Parameters:
		timeStamp:	(Output): The timestamp of the distance measurement
		distance:	(Output): The most recent distance measurement
	
	Returns: True if data was available
	
	Notes: 	Value updates approximately every 20ms so this function is likely 
			to return the same value if called more often than 20ms.
   ------------------------------------------------------------------------*/
bool LidarLite::getData(uint32_t &timeStamp, uint16_t &distance){ 
	bool gotData = false;
	//lock 
	pthread_mutex_lock(&dataMutex);
	//check if queues have data
	if(!dist_queue.empty()){
		//pop oldest data from buffer queue
		timeStamp = time_queue.front();
		distance = dist_queue.front();
		dist_queue.pop();
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
	Function: void LidarLite::setData(uint16_t dist)
	
	Description: Private class method to set the shared distance variable 
				 in a thread-safe way.
   
	Parameters: 
		dist:	(Input) Value to set the currently measured distance to.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/
void LidarLite::setData(uint32_t time, uint16_t dist){
	// lock/unlock mutex for thread safety
	pthread_mutex_lock(&dataMutex);
	time_queue.push(time);
	dist_queue.push(dist);
	pthread_mutex_unlock(&dataMutex);
}
	
/* ----------------------------------------------------------------------------
	Function: void* LidarLite::poll(void)
	
	Description: Private threaded function used to poll the LidarLite device. 
				 Updates the variable distance which contains the most recently
				 measured distance value.
   
	Parameters: None
	Returns: Nothing
	
	Notes: As this function is a pointer to a c++ class method, when using 
		   pthread_create it must be called using the static callPoll() method
		   with "this" passed in as a parameter. 
		   i.e. pthread_create(&pollThread, 0, &LidarLite::callPoll, this)
		   callPoll is defined in "lidar_lite.hpp"
   ------------------------------------------------------------------------*/
void* LidarLite::poll(void){
	printf("LidarLite thread: Started\n");
	bool success = true;

	uint32_t startTime = GetTickCount();
	uint32_t measurementCount = 0;
	
	//setup microsecond time
	struct timespec time_start;
	clock_gettime(CLOCK_REALTIME , &time_start);

	while(getSendStop() == 0){
		// send "measure" command
		success = I2Cdev::writeByte(LIDAR_LITE_ADRS, MEASURE_REG, MEASURE_VAL);

		if(success == false){
			printf("LidarLite error sending i2c measure command (%i = %s)\n",errno,strerror(errno)); 
			printf("Ensure the LidarLite is correctly connected to I2C\n");
		}else{
			//read distance bytes
			uint8_t distHI = 0;
			uint8_t distLO = 0;
 			int8_t numBytesHi = I2Cdev::readByte(LIDAR_LITE_ADRS,DISTANCE_REG_HI, &distHI);
			int8_t numBytesLo = I2Cdev::readByte(LIDAR_LITE_ADRS,DISTANCE_REG_LO, &distLO);
			
			//Check for I2C errors
			if (( numBytesHi == -1) || ( numBytesLo == -1)){
				printf("LidarLite error: Error receiving distance bytes from i2c\n");
			}else{
				//get microsecond time 
				struct timespec time_now;
				clock_gettime(CLOCK_REALTIME , &time_now);
				uint32_t us = calctimeus(&time_start, &time_now);
				//combine bytes and save to distance variable
				setData(us, (0xFFFF & (distHI << 8)) | distLO );
			}
			measurementCount++;
		}
		//usleep(50);
	}//end while(getSendStop() == 0)
		
	printf("LidarLite sensor thread: time:%d count:%d rate:%f\n", (GetTickCount() - startTime), measurementCount, 1.0 * measurementCount / (GetTickCount() - startTime) * 1000.0);
	
	printf("LidarLite thread: Stopped\n");
	pthread_exit(NULL);
}

/* ----------------------------------------------------------------------------
	Function: int LidarLite::getSendStop(void)
	
	Description: Private method to safely get the value of the stop command 
   
	Parameters: None
	Returns: The current value of the stop command
	
	Notes: Anything other than zero stops the polling thread
   ------------------------------------------------------------------------*/  
int LidarLite::getSendStop(void){
	int ret = 0;
	//Lock up the shared variable
	pthread_mutex_lock(&sendStopMutex);
	ret = sendStopIssued;
	pthread_mutex_unlock(&sendStopMutex);
	return ret;
}

/* ----------------------------------------------------------------------------
	Function: void LidarLite::setSendStop(int val) 
	
	Description: Private method to send a command to safely stop the polling thread
   
	Parameters: 
		val: (input) The value to write to the stop command. 
					 Anything other than 0 stops the thread.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/  
void LidarLite::setSendStop(int val) {
	//Lock up the shared variable
	pthread_mutex_lock(&sendStopMutex);
	sendStopIssued = val;
	pthread_mutex_unlock(&sendStopMutex);
}

/* ----------------------------------------------------------------------------
	Function: bool LidarLite::writeByte(uint8_t regAddr, uint8_t value)
	
	Description: Writes a byte to the fd I2C device file.

   ------------------------------------------------------------------------*/  
bool LidarLite::writeByte(uint8_t regAddr, uint8_t value){
	int8_t count = 0;
    uint8_t buf[2];
	int fd = -1;

    if (ioctl(fd, I2C_SLAVE, LIDAR_LITE_ADRS) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        return(false);
    }
    buf[0] = regAddr;
    memcpy(buf+1,&value,1);
    count = write(fd, buf, 2);
    if (count < 0) {
        fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
        return(false);
    } else if (count != 2) {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", 2, count);
        return(false);
    }

    return true;
}

/* ----------------------------------------------------------------------------
	Function: bool LidarLite::readByte(uint8_t regAddr, uint8_t *data)
	
	Description: reads a byte from the fd I2C device file.

   ------------------------------------------------------------------------*/  
int8_t LidarLite::readByte(uint8_t regAddr, uint8_t &data) {
	int fd = -1;
	//select i2c device
    if (ioctl(fd, I2C_SLAVE, LIDAR_LITE_ADRS) < 0) {
        printf("LidarLite error: Failed to select device: %s\n", strerror(errno));
		return false;
    }
	
    if (write(fd, &regAddr, 1) != 1) {
        printf("LidarLite error: Failed to write reg: %s\n", strerror(errno));
        return(-1);
    }
    uint8_t count = read(fd, &data, 1);
    if (count < 0) {
        printf("LidarLite error: Failed to read device: %s\n", strerror(errno));
        return(-1);
    }

    return count;
}

/* ----------------------------------------------------------------------------
	Function: uint32_t LidarLite::calctimeus(struct timespec *start, struct timespec *now)
	
	Description: Helper function with takes a start timespec and a current 
				 timespec and calcualtes the difference between them in 
				 microseconds

   ------------------------------------------------------------------------*/  
uint32_t LidarLite::calctimeus(struct timespec *start, struct timespec *now){
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