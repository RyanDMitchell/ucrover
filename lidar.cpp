/* ----------------------------------------------------------------------------
	File: lidar.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	25/08/2015
   
	File Description: This file implements a class which interfaces to a 
					  RPLidar device over a serial connection. 
	
	Notes: 	-The RPLidar is a low cost scanning laser device which provides 
				distance and angle measurements at about 5Hz rotational speed.
			-Generates about 20kBps of data at maximum acquisition rate
			-Uses the SDK provided by RPLidar downloaded from 
				http://www.slamtec.com/en-US/rplidar/index on 25/08/2015. 
			-This file is based on example code found in the SDK.
				The RPLidar SDK functions are declared in "rplidar.h" and
				the definitions are provided by linking to the pre-compiled 
				*.o files.

	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/

/* --system includes--*/   
#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"	  //Standard int types
#include <vector>	  //std::vector

/*--3rd party includes--*/
#include "rplidar.h"  //rplidar SDK header  
using namespace rp::standalone::rplidar;

/* --project includes--*/
#include "lidar.hpp"
#include "util.h"

/* ----------------------------------------------------------------------------
	Function: Lidar::Lidar(std::string port_name, unsigned int baud)
	
	Description: Constructor for the Lidar class
   
	Parameters:
		port_name: 	(Input) A string containing the path of the device file used for 
					 the underlying serial connection. i.e. "/dev/ttyUSB0"
		baud:		(Input) The speed of the serial connection.

	Returns: Nothing
	
	Notes: Uses the 3rd party RPLidar sdk
   ------------------------------------------------------------------------*/
Lidar::Lidar(std::string port_name, unsigned int baud){
	//init vars
	stopIssued = 0;
	this->port_name = port_name;
	this->baud = baud;
    // create the driver
	drv = NULL;
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!drv){
        printf("RPLidar error: Insufficient memory when creating RPLidar object\n");
    }
	//Initialize the mutexes
	pthread_mutex_init(&scanDataMutex,NULL);
	pthread_mutex_init(&stopMutex,NULL);
	
	//create the thread
	int result = pthread_create(&lidarThread, 0, &Lidar::callPollLidar, this);
	if (result != 0){
		printf("Lidar Error: Problem creating Lidar thread. Code: %i\n",result);
		pthread_detach(lidarThread);
	}
} 

/* ----------------------------------------------------------------------------
	Function: Lidar::~Lidar()
	
	Description: Destructor for the Lidar class, sends a command to stop the 
				 thread, waits for it to finish then cleans up 
				 the RPLidar driver
   
	Parameters: None
	Returns: Nothing
   ------------------------------------------------------------------------*/
Lidar::~Lidar(){
	this->stop();
	printf("Lidar: Closing RPLidar driver...\n");
	RPlidarDriver::DisposeDriver(drv);
	
	pthread_mutex_destroy(&stopMutex);
	pthread_mutex_destroy(&scanDataMutex);
	
	printf("Lidar: Safetly closed\n");
}

/* ----------------------------------------------------------------------------
	Function: void  Lidar::start()
	
	Description: Starts the scanning thread
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Does not check if a thread is already running, use carefully.
   ------------------------------------------------------------------------*/
void  Lidar::start(){
	//reset the stop command
	setStop(0);
	//create the thread
	int result = pthread_create(&lidarThread, 0, &Lidar::callPollLidar, this);
	if (result != 0){
		printf("Lidar Error: Problem creating Lidar thread. Code: %i\n",result);
		pthread_detach(lidarThread);
	}
}

/* ----------------------------------------------------------------------------
	Function: void  Lidar::stop()
	
	Description: Stops the scanning thread
   
	Parameters: None
	Returns: Nothing
	
	Notes: 	Does not check if a thread is already stopped, use carefully.
   ------------------------------------------------------------------------*/
void  Lidar::stop(){
	//set the stop flag to end the thread
	if(getStop() == 0){
		setStop(1);
		pthread_join(lidarThread, NULL);
	}
}

/* ----------------------------------------------------------------------------
	Function: Lidar::getScanData( std::vector<float> &angle, 
								  std::vector<float> &distance,
							      std::vector<uint8_t> &quality)
	
	Description: Accesses the most recent set of scan data read from the device 
				 and copies it to the passed in vectors
   
	Parameters:
		timeStamp:	(Output) The timestamp for the lidar scan data.
		angle: 		(Output) The vector where the scan angles are written. 
		distance: 	(Output) The vector where the scan distances are written.
		quality: 	(Output) The vector where the scan quality values are written.

	Returns: Nothing
	
	Notes: -The data are stored in ascending angle order to the vectors. 
		   -The size of each vector is guaranteed to be equal. 
		   -The vectors will have a size of zero if an error occurred. 
   ------------------------------------------------------------------------*/
bool Lidar::getScanData(uint32_t &timeStamp,float &angle,float &distance,uint8_t &quality){
	
	bool gotData = false;
	//lock 
	pthread_mutex_lock(&scanDataMutex);
	//check if queues have data
	if(!queue_time.empty()){
		//pop oldest data from buffer queue
		timeStamp = queue_time.front();
		angle = 	queue_angle.front();
		distance = 	queue_distance.front();
		quality = 	queue_quality.front();
		queue_time.pop();
		queue_angle.pop();
		queue_distance.pop();
		queue_quality.pop();
		gotData = true;
	}else{
		gotData = false;
	}
	//unlock
	pthread_mutex_unlock(&scanDataMutex);
	return gotData;
}

/* ----------------------------------------------------------------------------
	Function: Lidar::setScanData( std::vector<float> &angle, 
								  std::vector<float> &distance,
							      std::vector<uint8_t> &quality)
	
	Description: Private method to update the shared variables holding 
				 the scan data.
   
	Parameters:
		angle: 		(Input) The vector containing the scan angle values. 
		distance: 	(Input) The vector containing the scan distance values. 
		quality: 	(Input) The vector containing the scan quality values. 

	Returns: Nothing
   ------------------------------------------------------------------------*/
void Lidar::setScanData(std::vector<uint32_t> time,
						std::vector<float> angle, 
						std::vector<float> distance, 
						std::vector<uint8_t> quality){
							
	//Mutex for thread-safe access to the shared variables.
	pthread_mutex_lock(&scanDataMutex);
	for(unsigned int i = 0; i < time.size(); i++){
		queue_time.push(time[i]);
		queue_angle.push(angle[i]);
		queue_distance.push(distance[i]);
		queue_quality.push(quality[i]);
	}
	pthread_mutex_unlock(&scanDataMutex);
}

/* ----------------------------------------------------------------------------
	Function: void* Lidar::pollLidar(void)
	
	Description: Private threaded function used to read scan data from the 
				 RPLidar device. Updates the shared scan data variables every 
				 time it gets a new scan.
   
	Parameters: None
	Returns: Nothing
	Notes: -The data are copied in ascending angle order to the vectors.
			-The size of each vector is guaranteed to be equal. 
			-If an error reading the lidar occurs the thread will exit.
   ------------------------------------------------------------------------*/
void* Lidar::pollLidar(void){
	printf("RPLidar thread: Started\n");
	bool lidarError = false;
	
	
	printf("RPLidar thread: Initialising Lidar...\n");
	
	// Connect to the lidar
	if (IS_FAIL(drv->connect(port_name.c_str(), baud))) {
		printf("RPLidar error:  Cannot bind to the specified serial port %s. Check port is correct and RPLidar is connected\n", port_name.c_str());
	}
	//start the RPlidar background task to get lidar data
	drv->startScan(true);
	printf("RPLidar thread: ...Lidar initialised. Getting scan data\n");
	
	struct timespec scan_start;
	struct timespec scan_stop;
	struct timespec time_start;
	clock_gettime(CLOCK_REALTIME , &time_start);
		
	while(getStop() == 0 && !lidarError){
		// Get scan data
		u_result op_result;
		rplidar_response_measurement_node_t nodes[400];
		size_t   count = _countof(nodes);
		//timeit
		clock_gettime(CLOCK_REALTIME , &scan_start);
        op_result = drv->grabScanData(nodes, count, 200);
        clock_gettime(CLOCK_REALTIME , &scan_stop);
		
		uint32_t startus = calctimeus(&time_start,&scan_start);
		uint32_t stopus = calctimeus(&time_start,&scan_stop);
		
		//If successful extract the relevant parts of the raw data
		if (IS_OK(op_result)) {
			drv->ascendScanData(nodes, count);
			//temporary variables
			std::vector<float> tempAngle;
			std::vector<float> tempDistance;
			std::vector<uint8_t> tempQuality;
			std::vector<uint32_t> tempTime;
			tempAngle.reserve(360);
			tempDistance.reserve(360);
			tempQuality.reserve(360);
			tempTime.reserve(360);
			//copy the scan data into the temp vars
			for (int pos = 0; pos < (int)count ; ++pos) {
				//only save the non-zero quality measurements to save resources
				if(nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 0){
					//linearly interpolate the time
					uint32_t time = startus + pos*(stopus-startus)/count;
					tempTime.push_back(time);					
					tempQuality.push_back(nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
					tempAngle.push_back((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
					tempDistance.push_back(nodes[pos].distance_q2/4.0f);
				}
			}
			//threadsafely update shared vars holding the scan data
			setScanData(tempTime,tempAngle, tempDistance, tempQuality);
		}else{
			//Print error msg
 			printf("RPLidar Error: Could not get scan data from a frame. ");
			if(op_result == RESULT_ALREADY_DONE){printf("Code: RESULT_ALREADY_DONE\n");}
			if(op_result == RESULT_INVALID_DATA){printf("Code: RESULT_INVALID_DATA\n");}
			if(op_result == RESULT_OPERATION_FAIL){printf("Code: RESULT_OPERATION_FAIL\n");}
			if(op_result == RESULT_OPERATION_TIMEOUT){printf("Code: RESULT_OPERATION_TIMEOUT\n");}
			if(op_result == RESULT_OPERATION_STOP){printf("Code: RESULT_OPERATION_STOP\n");}
			if(op_result == RESULT_OPERATION_NOT_SUPPORT){printf("Code: RESULT_OPERATION_NOT_SUPPORT\n");}
			if(op_result == RESULT_FORMAT_NOT_SUPPORT){printf("Code: RESULT_FORMAT_NOT_SUPPORT\n");}
			if(op_result == RESULT_INSUFFICIENT_MEMORY){printf("Code: RESULT_INSUFFICIENT_MEMORY\n");}

			//try restting
			printf("RPLidar: Resetting device...\n");
			drv->stop();
			//flush serial 
			rplidar_response_measurement_node_t flush_nodes[1];
			size_t   flushcount = _countof(nodes);
			while(IS_OK(drv->grabScanData(flush_nodes, flushcount,0))){
				//spin it
			}
			drv->startScan(true);
			printf("RPLidar: Device reset\n");
		}
		//sleep to limit rate
		//usleep(200000);
	}//end while(getStop() == 0)
	
	//drv->stop();
	printf("RPLidar thread: Lidar scan thread closed\n");	
	pthread_exit(NULL);
}

/* ----------------------------------------------------------------------------
	Function: int Lidar::getStop(void)
	
	Description: Private method to safely get the value of the stop command 
   
	Parameters: None
	Returns: The current value of the stop command
	
	Notes: Anything other than zero stops the polling thread
   ------------------------------------------------------------------------*/  
int Lidar::getStop(void){
	int ret = 0;
	//Lock up the shared variable
	pthread_mutex_lock(&stopMutex);
	ret = stopIssued;
	pthread_mutex_unlock(&stopMutex);
	return ret;
}

/* ----------------------------------------------------------------------------
	Function: void Lidar::setStop(int val)
	
	Description: Private method to send a command to safely stop the lidar thread
   
	Parameters: 
		val: (input) The value to write to the stop command. 
					 Anything other than 0 stops the thread.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/  
void Lidar::setStop(int val){
	//Lock up the shared variable
	pthread_mutex_lock(&stopMutex);
	stopIssued = val;
	pthread_mutex_unlock(&stopMutex);
}

/* ----------------------------------------------------------------------------
	Function: uint32_t Lidar::calctimeus(struct timespec *start, struct timespec *now)
	
	Description: Helper function with takes a start timespec and a current 
				 timespec and calcualtes the difference between them in 
				 microseconds

   ------------------------------------------------------------------------*/  
uint32_t Lidar::calctimeus(struct timespec *start, struct timespec *now){
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

/* --------------------------------------------------------------------------
	Function: int Lidar::getQueueSize()
	
	Description: Method to determine how large the buffer is
	Returns: size of the buffer
   ------------------------------------------------------------------------*/  
int Lidar::getQueueSize() {
	//don't call too often
	int size = 0;
	pthread_mutex_lock(&scanDataMutex);
	size = queue_time.size();
	//safety check
	if(queue_time.size() != queue_distance.size()){
		printf("Lidar thread error: queue size mismatch!!\n");
	}
	pthread_mutex_unlock(&scanDataMutex);
	return size;
}