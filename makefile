#/* ----------------------------------------------------------------------------
#	File: makefile
#
#	Author: Ryan Mitchell
#	Email:	rdm86@uclive.ac.nz
#	Date:	25/08/2015
#  
#	File Description: This is the makefile for the mapping project. 
#					  It performs the compilation on the Raspberry Pi. 
#	
#	Notes: 	-The project is manually linked against the pre-compiled RPLidar SDK *.o files. 
#				Specifically the four files (rplidar_driver.o thread.o net_serial.o timer.o)
#				
#	Purpose: This file is part of the rover mapping system designed to 
#		generate a map of an underfloor environment. This software is 
#		designed to run on a Raspberry Pi located on the rover. The 
#		purpose of this software is to pre-process and stream data from a 
#		variety of sensors over a wireless serial link. The computer at the 
#		other end of the wireless link will handle the map generation.
#   --------------------------------------------------------------------------*/

RPLIDAR_SDK = $(CURDIR)/rplidar_sdk

CC=g++
CFLAGS=-c -Wall -D_REENTRANT
INCLUDES=-I $(RPLIDAR_SDK)
LDFLAGS= -lboost_system -lpthread -lrt -lwiringPi -lmpg123 -lao
SOURCES=main.cpp serial.cpp util.cpp yei.cpp yei_heading.cpp lidar.cpp lidar_lite.cpp I2Cdev.cpp filer.cpp encoder.cpp servo.cpp
OBJECTS=$(SOURCES:.cpp=.o) 
EXECUTABLE=mapping


all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) rplidar_driver.o thread.o net_serial.o timer.o -o $@
	
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@ $(INCLUDES)
	
clean: 
	rm $(OBJECTS)

