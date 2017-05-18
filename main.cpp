/* ----------------------------------------------------------------------------
	File: main.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	24/08/2015
   
	File Description:
		Contains the main function. Instantiates and calls 
		all the relevant objects and methods.
		
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/

 /* --system includes --*/
#include <stdio.h>		//printf
#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>		//usleep
#include <cerrno>		//error handling
#include <stdlib.h>		//RNG functions and system
#include <signal.h>		//terminate signals
#include "stdint.h"
#include <time.h>

/* --project includes-- */
#include "util.h"
#include "filer.hpp"
#include "servo.hpp"

//keyboard terminate flag
sig_atomic_t stopFlag = 0;

/* ----------------------------------------------------------------------------
	Function: void signalHandler(int signum)
	
	Description: Catches the keyboard terminate interupt (ctrl+c) and safely exits
   
	Parameters:
		signum:		(input) The signal ID recieved
	
	Returns: N/A
   ------------------------------------------------------------------------*/
   void signalHandler(int signum){
		stopFlag = 1;
   }
   
/* ----------------------------------------------------------------------------
	Function: int main(int argc, char *argv[])
	
	Description: Instantiates and calls all the relevant objects and methods 
		to set up streaming of sensor data.
   
	Parameters:
	
	Returns: N/A
   ------------------------------------------------------------------------*/
int main(int argc, char *argv[]){
	 
	uint32_t runTime = 10*60*1000;
	
	//specify the sensor ports
	std::string yei1_port = "/dev/ttyYEI1"; 
	std::string yei2_port = "/dev/ttyYEI2";
	std::string lidar_port = "/dev/ttyLIDAR";
	std::string encoder_port = "/dev/ttyENCODER";
	std::string servo_port = "/dev/ttySERVO";
	
	//setup the servo
 	Servo servo(servo_port,9600);
	int servoPos = 0;
	int zeroPos = 1400;
	int leftPos = 1800;
	int rightPos = 1000; 
	uint32_t servoPeriod = 10000;
	servo.setPos(zeroPos);

	printf("------------------------------------------\n");
	printf("Main: Getting sensor data for %.1f seconds\n",runTime/1000.0f);
	printf("------------------------------------------\n");
	
	//Start the data filer
	Filer filer(yei1_port, yei2_port, lidar_port, encoder_port);
	filer.startFiling();

	//Register the signal handler(after creating everything) 
	//for catching the ctrl+c keyboard command
	signal(SIGINT, &signalHandler);
	
	uint32_t prevTime = GetTickCount();
	uint32_t startTime = GetTickCount();
	usleep(50000);
	//make sure the servo is set
	servo.setPos(zeroPos);
	
	//main command control loop
	while(GetTickCount() - startTime < runTime && stopFlag == 0){
/* 		//Scan the servo
  		if(GetTickCount() - prevTime > servoPeriod){
			if(servoPos == 0){
				servoPos = 1;
				servo.setPos(rightPos);
			}else if(servoPos == 1){
				servoPos = 0;
				servo.setPos(leftPos);
			}
			prevTime = GetTickCount();
		} */
		
		//don't jam up the mutexes
		usleep(10000);
	}
	filer.stopFiling();
	if(stopFlag == 1){
		printf("Main: Caught keyboard interupt, exiting safely...\n");
	}
	usleep(1000000);
	printf("Main: Exited\n");
	return 0; 
}
   