/* ----------------------------------------------------------------------------
	File: filer.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	26/08/2015
   
	File Description: Files the sensor data
				
	Notes: 
				
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
#include "stdint.h"		//Standard int types
#include <pthread.h> 	//multithreading
#include <sys/time.h>	//time of day

 /* --project includes --*/
#include "filer.hpp"
#include "util.h"
#include "yei.hpp"
#include "lidar.hpp"
#include "lidar_lite.hpp"

/* ----------------------------------------------------------------------------
	Function: Filer::Filer()
	
	Description: Constructor for the Filer Class. Attempts to open all the 
				 implemented sensor objects.
   
	Parameters: None
	Returns: Nothing
   ------------------------------------------------------------------------*/
Filer::Filer(std::string yei1Port, std::string yei2Port, std::string lidarPort, std::string encoderPort){
	//select sensors
	useYei1 	= true;
	useYei2 	= true;
	useLidar 	= false;
	useLaser	= true;
	useEncoder 	= true;

	//init the vars
	stopIssued = 1;
	startTime = 0;
	laserMeasurements = 0;
	lidarMeasurements = 0;
	IMU1Measurements = 0;
	IMU2Measurements = 0; 
	encoderMeasurements = 0;
	
	//save start time
	startTime = GetTickCount();
	
	//Initialize the mutexes
	pthread_mutex_init(&stopMutex,NULL);
	
	//create all the sensor objects
	if(useYei1) yei1 = new Yei(yei1Port,115200);
	if(useYei2) yei2 = new Yei_heading(yei2Port,115200);
	if(useLidar) lidar = new Lidar(lidarPort,115200);
	if(useLaser) laser = new LidarLite();
	if(useEncoder) encoder = new Encoder(encoderPort,19200);

	//start streaming data from yei
	/*Timing values for reference
		0x00004E20 == 20000uS/50Hz
		0x00002710 == 10000uS/100Hz
		0x00000FA0 == 4000uS/250Hz
		0x00000DF3 == 3571uS/280Hz	  
		0x00000C35 == 3125uS/320Hz
		0x000009C4 == 2500uS/400Hz
		0x000007D0 == 2000uS/500Hz 
		0x00000640 == 1600uS/640Hz		
		0x000004E2 == 1250uS/800Hz  
		0x000003E8 == 1000uS/1000Hz
		(Update rate, duration, delay)*/
		
	if(useYei1) yei1->start(0x000009C4,0xFFFFFFFF,0x00000000);
	if(useYei2) yei2->start(0x00002710,0xFFFFFFFF,0x00000000);
}

/* --------------------------------------------------------------------------
	Function: Filer::~Filer()
	
	Description: Destructor for the Filer class. Stops the thread and 
				 closes all the sensor objects.
   
	Parameters: None
	Returns: Nothing
   ------------------------------------------------------------------------*/
Filer::~Filer(){
	//stop the thread
	setStop(1);
	pthread_join(fileThread, NULL);
	
	printf("Filer: Safetly closed\n");
}

/* --------------------------------------------------------------------------
	Function: int Filer::getStop(void)
	
	Description: Private method to safely get the value of the stop command 
   
	Parameters: None
	Returns: The current value of the stop command
	
	Notes: Anything other than zero stops the streaming thread 
   ------------------------------------------------------------------------*/  
int Filer::getStop(void){
	int ret = 0;

	pthread_mutex_lock(&stopMutex);
	ret = stopIssued;
	pthread_mutex_unlock(&stopMutex);
	return ret;
}


/* --------------------------------------------------------------------------
	Function: void Filer::setStop(int val)
	
	Description: Private method to send a command to safely stop the 
				 streaming thread
   
	Parameters: 
		val: (input) The value to write to the stop command. 
					 Anything other than 0 stops the thread.
	
	Returns: Nothing
   ------------------------------------------------------------------------*/  
void Filer::setStop(int val) {
	pthread_mutex_lock(&stopMutex);
	stopIssued = val;
	pthread_mutex_unlock(&stopMutex);
}


/* --------------------------------------------------------------------------
	Function: void* Filer::startStreaming()
	
	Description: Starts the streaming thread
				 
	Parameters: None
	Returns: Nothing
	
   ------------------------------------------------------------------------*/
void Filer::startFiling(){
	//Start the thread if it is not running already
	//Any value other than zero from getStop() indicates the thread is stopped
	if(getStop() != 0){	
		printf("Filer: Received start command\n");
		//create the thread
		int result = pthread_create(&fileThread, 0, &Filer::callFile, this);
		if (result != 0){
			printf("Filer Error: Problem creating file thread. Code: %i\n",result);
			pthread_detach(fileThread);
		}else{
			//if it was successful reset the stop command
			setStop(0);	
		}
	}else{
		printf("Filer: Received start command when already filing\n");
	}
}

/* --------------------------------------------------------------------------
	Function: void* Filer::stopStreaming()
	
	Description: Stops the streaming thread
				 
	Parameters: None
	Returns: Nothing
	
   ------------------------------------------------------------------------*/
void Filer::stopFiling(){
	// Anything other than zero stops the thread
	if(getStop() == 0){
		printf("Filer: Received stop command\n");
		setStop(1);
	}else{
		printf("Filer: Received stop command while already stopped\n");
	}
}

/* --------------------------------------------------------------------------
	Function: void* Filer::stream(void)
	
	Description: Private threaded function used to stream data from multiple
				 sensor devices over an TCP link.
				 
	Parameters: None
	Returns: Nothing
	
	Notes: 
   ------------------------------------------------------------------------*/
void* Filer::file(void){
	printf("Filer thread: Started\n");

	//data storage for filing
	std::vector<std::string> IMU1Data;
	std::vector<std::string> IMU2Data;
	std::vector<std::string> laserData;
	std::vector<std::string> encoderData;
	std::vector<std::string> lidarData;
	std::vector<float> lidarAngleData;
	std::vector<float> lidarDistanceData;
	std::vector<uint8_t> lidarQualityData;
	std::vector<uint32_t> lidarTimeData;
	std::vector<uint32_t> IMU1TimeStampData;
	std::vector<float> IMU1SensorData;
	std::vector<uint32_t> IMU2TimeStampData;
	std::vector<float> IMU2SensorData;
	std::vector<uint32_t> laserTimeStampData;
	std::vector<uint16_t> laserDistData;
	std::vector<uint32_t> encoderTimeStampData;
	std::vector<int32_t> encoderLData;
	std::vector<int32_t> encoderRData;


	if(useYei1){
		std::string IMU1Title ("at mS,ax g,ay g,az g,gx rad/s,gy rad/s,gz rad/s, mx gauss, my gauss, mz gauss\n");
		IMU1Data.push_back(IMU1Title);
		//cumlative data variables
		IMU1Data.reserve(5000000);
		IMU1TimeStampData.reserve(5000000);
		IMU1SensorData.reserve(5000000);
	}
	if(useYei2){
		std::string IMU2Title ("at mS,pitch,yaw,roll\n");
		IMU2Data.push_back(IMU2Title);
		//cumlative data variables
		IMU2Data.reserve(500000);
		IMU2TimeStampData.reserve(500000);
		IMU2SensorData.reserve(500000);
	}
	if(useLaser){
		std::string laserTitle ("time_laser mS, dist_laser mm\n");
		laserData.push_back(laserTitle);
		//cumlative data variables
		laserData.reserve(500000);
		laserTimeStampData.reserve(500000);
		laserDistData.reserve(500000);
	}
	
	if(useEncoder){
		std::string encoderTitle ("time_en mS,left_en count,right_en count\n");
		encoderData.push_back(encoderTitle);
		encoderData.reserve(500000);
		//cumlative data variables
		encoderTimeStampData.reserve(500000);
		encoderLData.reserve(500000);
		encoderRData.reserve(500000);
	}
	
	//save lidar data raw before converting to strings as it is slow to loop 
	//thorugh and convert during runtime
	//string variables
	if(useLidar){
		std::string lidarTitle ("t,angle,distance,quality\n");
		lidarData.push_back(lidarTitle);
		lidarData.reserve(10000000);
		//cumlative data variables
		lidarAngleData.reserve(10000000);
		lidarDistanceData.reserve(10000000);
		lidarQualityData.reserve(10000000);
		lidarTimeData.reserve(10000000);
	}

	//Save the IMU data as a vector of std::strings in memory and write out to a file once 
	//mapping is stopped.
	startTime = GetTickCount();
	uint32_t statsPrintTime = startTime;
	
	//print the sensor selection info
	printf("Sensor selections: Lidar: %s Laser: %s Encoders: %s IMU1: %s IMU2: %s\n",
			useLidar?"On":"Off",useLaser?"On":"Off",useEncoder?"On":"Off",useYei1?"On":"Off",useYei2?"On":"Off");
			
	while(getStop() == 0){

		
		//update the current data
		//Note: Avoid spamming the queueSize() checker or isready() checkers
		if(useYei1){
			std::vector<float> yei1Data;
			uint32_t yei1TimeStamp;
			if(yei1->getData(yei1TimeStamp, yei1Data)){
				IMU1TimeStampData.push_back(yei1TimeStamp);
				IMU1SensorData.insert(IMU1SensorData.end(),yei1Data.begin(),yei1Data.end());
				IMU1Measurements++;
			}else{
				//Print the filing stats
				if(GetTickCount() - statsPrintTime > 1000){
					statsPrintTime = GetTickCount();
					float timeElapsed = (GetTickCount() - startTime) / 1000.0;
					printf("Current data: (%3.2f) Yei1:%4i(%2.1f) Yei2:%4i(%2.1f) encoder:%4i(%2.1f) laser:%4i(%2.1f) lidar:%4i(%2.1f)\n",timeElapsed,IMU1Measurements,IMU1Measurements/timeElapsed,IMU2Measurements, IMU2Measurements/timeElapsed,  encoderMeasurements, encoderMeasurements/timeElapsed, laserMeasurements, laserMeasurements/timeElapsed, lidarMeasurements,lidarMeasurements/timeElapsed);
				}
			}
		}//end useYei1
		if(useYei2){
			std::vector<float> yei2Data;
			uint32_t yei2TimeStamp;
			if(yei2->getData(yei2TimeStamp, yei2Data)){
 				IMU2TimeStampData.push_back(yei2TimeStamp);
				IMU2SensorData.insert(IMU2SensorData.end(),yei2Data.begin(),yei2Data.end());
				IMU2Measurements++;
			}
		}//end useYei2
		if(useEncoder){
			uint32_t enTimeStamp;
			int32_t enLeftCount;
			int32_t enRightCount;
			if(encoder->getData(enTimeStamp, enLeftCount, enRightCount)){
				encoderTimeStampData.push_back(enTimeStamp);
				encoderLData.push_back(enLeftCount);
				encoderRData.push_back(enRightCount);
				encoderMeasurements++;
			}
		}//end use Encoder
		if(useLaser){
			uint16_t laserDist;
			uint32_t laserTimeStamp;
			if(laser->getData(laserTimeStamp, laserDist)){
				laserTimeStampData.push_back(laserTimeStamp);
				laserDistData.push_back(laserDist);
				laserMeasurements++;
			}
		}//end useLaser
		if(useLidar){
			uint8_t quality;
			float angle;
			float distance;
			uint32_t lidarTimeStamp;
			if(lidar->getScanData(lidarTimeStamp, angle, distance, quality)){
				//save the data for later processing
				lidarAngleData.push_back(angle);
				lidarDistanceData.push_back(distance);
				lidarQualityData.push_back(quality);
				lidarTimeData.push_back(lidarTimeStamp);
				lidarMeasurements++;
			}
		}//end uselidar

	}//end while(getStop() == 0)
	
	//stop the sensors
	if(useLaser){
		printf("Filer: Stopping laser...\n"); 
		laser->stop();
	}
	if(useEncoder){
		printf("Filer: Stopping encoder...\n"); 
		encoder->stop();
	}
	if(useYei1){
		printf("Filer: Stopping yei1...\n"); 
		yei1->stop();
	}
	if(useYei2){
		printf("Filer: Stopping yei2...\n"); 
		yei2->stop();
	}
	if(useLidar){
		printf("Filer: Stopping lidar...\n"); 
		lidar->stop();
	}

	printf("Filer thread: Preparing to write data to files...\n");
	//get the time as a string to use in the filenames
	struct timeval tv;
	struct tm *nowtm;
	gettimeofday(&tv, NULL);
	time_t nowtime = tv.tv_sec;
	nowtm = localtime(&nowtime);
	
	if(useYei1){
		//construct the filename
		char IMU1NameBuff[64];
		strftime(IMU1NameBuff, sizeof IMU1NameBuff, "logs/%Y_%m_%d_%H_%M_%S_IMU1.csv", nowtm);
		std::string IMU1Filepath(IMU1NameBuff);
		
		//process the yei data
		for(unsigned int i = 0; i < IMU1TimeStampData.size() ; i++){
			char buff[350];	
			int ind = i*9;
			sprintf(buff,"%i,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
						 IMU1TimeStampData[i], IMU1SensorData[ind+0], IMU1SensorData[ind+1], IMU1SensorData[ind+2],
										       IMU1SensorData[ind+3], IMU1SensorData[ind+4], IMU1SensorData[ind+5],
											   IMU1SensorData[ind+6], IMU1SensorData[ind+7], IMU1SensorData[ind+8]);
			std::string temp = buff;
			IMU1Data.push_back(temp);
		}
		
		//write the data stored in memory out to file
		writeDataToFile(IMU1Filepath, IMU1Data);
	}
	if(useYei2){
		//construct the filename
		char IMU2NameBuff[64];
		strftime(IMU2NameBuff, sizeof IMU2NameBuff, "logs/%Y_%m_%d_%H_%M_%S_IMU2.csv", nowtm);
		std::string IMU2Filepath(IMU2NameBuff);
		
		//process the yei data
		for(unsigned int i = 0; i < IMU2TimeStampData.size() ; i++){
			char buff[200];	
			int ind = i*3;
			sprintf(buff,"%i,%f,%f,%f\n", 
						 IMU2TimeStampData[i], IMU2SensorData[ind+0], IMU2SensorData[ind+1], IMU2SensorData[ind+2]);
			std::string temp = buff;
			IMU2Data.push_back(temp);
		}
		
		//write the data stored in memory out to file
		writeDataToFile(IMU2Filepath, IMU2Data);
	}
	if(useLidar){
		//construct the filename
		char lidarNameBuff[64];
		strftime(lidarNameBuff, sizeof lidarNameBuff, "logs/%Y_%m_%d_%H_%M_%S_Lidar.csv", nowtm);
		std::string lidarFilepath(lidarNameBuff);
		
		//process the lidar data
		//this is really slow 
		//copy the lidar data as a string into memory
		for(unsigned int i = 0; i < lidarAngleData.size() ; i++){
			char buff[100];	
			sprintf(buff,"%i,%f,%f,%i\n",lidarTimeData[i],lidarAngleData[i],lidarDistanceData[i],lidarQualityData[i]);
			std::string temp = buff;
			lidarData.push_back(temp);
		}
	
		//write lidar out to file
		writeDataToFile(lidarFilepath, lidarData);
	}
	if(useLaser){
		//construct the filename
		char laserNameBuff[64];
		strftime(laserNameBuff, sizeof laserNameBuff, "logs/%Y_%m_%d_%H_%M_%S_Laser.csv", nowtm);
		std::string laserFilepath(laserNameBuff);
		
		//process the laser data
		for(unsigned int i = 0; i < laserTimeStampData.size() ; i++){
			char buff[100];	
			sprintf(buff,"%i,%i\n", laserTimeStampData[i], laserDistData[i]);
			std::string temp = buff;
			laserData.push_back(temp);
		}
		
		//write the data stored in memory out to file
		writeDataToFile(laserNameBuff, laserData);
	}
	if(useEncoder){
		//construct the filename
		char encoderNameBuff[64];
		strftime(encoderNameBuff, sizeof encoderNameBuff, "logs/%Y_%m_%d_%H_%M_%S_Encoder.csv", nowtm);
		std::string encoderFilepath(encoderNameBuff);
		
		//process the data
		for(unsigned int i = 0; i < encoderTimeStampData.size() ; i++){
				char buff[250];		
				sprintf(buff,"%i,%i,%i\n", 
							 encoderTimeStampData[i], encoderLData[i], encoderRData[i]);
				std::string temp = buff;
				encoderData.push_back(temp);
		}

		//write the data stored in memory out to file
		writeDataToFile(encoderFilepath, encoderData);
	}
	
	//print the data stats
	printf("----------- Filed data stats -------------\n");
	printf("%i lidar lines\n", lidarData.size() - 1);
	printf("%i IMU1 lines\n", IMU1Data.size() -1);
	printf("%i IMU2 lines\n", IMU2Data.size() -1);
	printf("%i encoder lines\n", encoderData.size() -1);
	printf("%i laser lines\n", laserData.size() -1);
	printf("------------------------------------------\n");


	printf("Filer thread: Stopped\n");
	pthread_exit(NULL);
}

/* --------------------------------------------------------------------------
	Function: void Filer::writeDataToFile(std::string filePath, std::vector<std::string> data)
	
	Description: Private method to write data to a file
   
	Parameters: 
		filePath:	(input): The desired filename/filepath of the file
		data:		(input): The vector of string to write to the file
		
	Returns: Nothing
   ------------------------------------------------------------------------*/  
void Filer::writeDataToFile(std::string filePath, std::vector<std::string> data){
	//write the data stored in memory out to a file
	FILE *pFile;
    pFile = fopen(filePath.c_str(),"w");
	for(unsigned int j = 0; j < data.size(); j ++){
		fprintf(pFile,"%s",data[j].c_str());
	}
	fclose(pFile);	
}