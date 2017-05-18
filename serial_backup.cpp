/* ----------------------------------------------------------------------------
	File: serial.cpp

	Author: Ryan Mitchell
	Email:	rdm86@uclive.ac.nz
	Date:	24/08/2015
   
	File Description: This file implements a serial connection with a specified 
		port and baud using termios. The serial connection is statically set to 
		8N1 with CTSRTS enabled.
	
	Note: All functions are blocking.
	
	Purpose: This file is part of the rover mapping system designed to 
		generate a map of an underfloor environment. This software is 
		designed to run on a Raspberry Pi located on the rover. The 
		purpose of this software is to pre-process and stream data from a 
		variety of sensors over a wireless serial link. The computer at the 
		other end of the wireless link will handle the map generation.
   --------------------------------------------------------------------------*/

 /* --system includes --*/
#include <stdio.h>
#include <cstring>		//strncmp
#include <string>		//std::string
#include <unistd.h>		//Used for UART
#include <fcntl.h>		//Used for UART
#include <termios.h>	//Used for UART
#include <stdlib.h>		//RNG functions
#include "stdint.h"		//Standard int types
#include <cerrno>		//error handling
#include <sys/ioctl.h> 	//ioctl command

/* --project includes-- */
#include "serial.hpp"
#include "util.h"

/* ----------------------------------------------------------------------------
	Function: Serial(std::string port_name, unsigned int baud)
	
	Description: Constructor for the Serial Class
   
	Parameters:
		port_name: 	(Input) A string containing the path of the device file used for 
					the serial connection. i.e. "/dev/ttyUSB0"
		baud:		(Input) The speed of the serial connection. Must be one of the 
					following set of bauds (1200, 2400, 4800, 9600, 19200,
					38400, 57600, 115200, 230400, 460800). If not from this 
					list the default of 115200 will be used.
	Returns: Nothing
   ------------------------------------------------------------------------*/
Serial::Serial(std::string port_name,unsigned int baud){

	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	
	//Open UART in non blocking read/write mode
	uart0_filestream = -1;
	uart0_filestream = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);		
	if (uart0_filestream == -1){
		printf("Serial Error: Unable to open UART. Ensure the the port is correct and it is not in use by another application\n");
	}else{
		//Get the baud rate flag from the passed in baud parameter
		int BAUD = 0;
		const unsigned int DEFAULT_BAUD = B115200;
		const unsigned int _DEFAULT_BAUD = 115200;
		switch(baud){
			case 1200: BAUD = B1200; break;
			case 2400: BAUD = B2400; break;
			case 4800: BAUD = B4800; break;
			case 9600: BAUD = B9600; break;
			case 19200: BAUD = B19200; break;
			case 38400: BAUD = B38400; break;
			case 57600: BAUD = B57600; break;
			case 115200: BAUD = B115200; break;
			case 230400: BAUD = B230400; break;
			case 460800: BAUD = B460800; break;
			default: printf("Serial: Unknown baud rate %i. Setting to default %i\n",baud,_DEFAULT_BAUD);BAUD = DEFAULT_BAUD; break;
		}
		//CONFIGURE THE UART
		//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
		//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
		//	CSIZE:- CS5, CS6, CS7, CS8
		//	CLOCAL - Ignore modem status lines
		//	CREAD - Enable receiver
		//	IGNPAR = Ignore characters with parity errors
		//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
		//	PARENB - Parity enable
		//	PARODD - Odd parity (else even)
		//  CRTSCTS - Enable cts/rts flow control
		
		struct termios options;
		tcgetattr(uart0_filestream, &options);
		options.c_cflag = BAUD | CS8 | CLOCAL | CREAD | CRTSCTS;
		options.c_iflag = 0;
		options.c_oflag = 0;
		options.c_lflag = 0;
		usleep(100);
		tcflush(uart0_filestream, TCIFLUSH);
		tcsetattr(uart0_filestream, TCSANOW, &options);
		
		//consume all bytes
		uint8_t dummy = 0;
		while(read(uart0_filestream, &dummy, 1) == 1){
			//spin it until all leftover bytes are consumed
		}
	}
}

/* ----------------------------------------------------------------------------
	Function: int Serial::send(uint8_t *data, uint16_t numBytes)
	
	Description: Sends one or more bytes over the serial connection
   
	Parameters:
		data: 		(Input) An array of bytes to be sent over the serial port
		numBytes: 	(Input) The number of bytes stored in the data array
					
	Returns: The number of bytes sent or -1 if could not connect to the UART
	
	Notes: 	Function is blocking. If there is a UART write error then a 
			message will be printed.
   ------------------------------------------------------------------------*/
int Serial::send(uint8_t *data, uint16_t numBytes){
	int bytesSent = -1;
	bool sendError = false;
	if (uart0_filestream != -1){
		bytesSent = 0;
		while((bytesSent < numBytes) && (!sendError)){
			int count = write(uart0_filestream, &data[bytesSent], 1);
			int errval = errno;
			if(count != 1){
				if(errval == 11){
					//error 11 => "Resource temporarily unavailable" 
					//This means the serial port is not ready; spin while loop
				}else{
					printf("Serial error: serial send error(%i = %s)\n",errval,strerror(errval));
					sendError = true;
				}
			}else{
				bytesSent++;
			}
		}
	}
	return bytesSent;
}

/* ----------------------------------------------------------------------------
	Function: int Serial::get(uint8_t *data, uint16_t numBytes)
	
	Description: Reads one or more bytes from the serial connection
   
	Parameters:
		data: 		(Output) A pointer to where the output data will be stored
		numBytes: 	(Input) The number of bytes to read
					
	Returns: The number of bytes read or -1 if could not connect to the UART
	
	Notes: 	Function is blocking. The read will timeout if it has been longer 
			than the value set by READTIMEOUT. If there is a UART write error 
			or a timeout then a message will be printed.
   ------------------------------------------------------------------------*/
int Serial::get(uint8_t *data, uint16_t numBytes){
	int bytesReceived = -1;
	if (uart0_filestream != -1){
		bytesReceived = 0;
		int startTime = GetTickCount();
		bool timeout = false;
		while(bytesReceived < numBytes && !timeout){
			int count = read(uart0_filestream, &data[bytesReceived], 1);
			int errval = errno;
			if(count != 1){
				if(errval == 11){
					//error 11 => "Resource temporarily unavailable" 
					//This means the serial port is not ready; spin while loop

				}else if (errval == 0){
					// error 0 => "Successful". This means no data was available
				}else{
					printf("Serial error: serial receive error(%i = %s)\n",errval,strerror(errval));
				}
			}else{
				bytesReceived++;
			}
			if(GetTickCount() - startTime > READTIMEOUT){
				printf("Serial error: read timed out(%ims)\n",GetTickCount() - startTime);
				timeout = true;
			}
		}
	}
	return bytesReceived;
}

/* ----------------------------------------------------------------------------
	Function: int Serial::getline(uint8_t *data)
	
	Description: Reads bytes from the serial connection untill a newline 
				 character is received.
   
	Parameters:
		data: 	(Output) A pointer to where the output data will be stored
					
	Returns: The number of bytes read or -1 if could not connect to the UART
	
	Notes: 	Function is blocking. The read will timeout if it has been longer 
			than the value set by READTIMEOUT. If there is a UART write error 
			or a timeout then a message will be printed.
   ------------------------------------------------------------------------*/
int Serial::getline(uint8_t *data){
	int bytesReceived = -1;
	bool newline = false;
	bool timeout = false;
	if (uart0_filestream != -1){
		bytesReceived = 0;
		int startTime = GetTickCount();
		while(!newline && !timeout){
			int count = read(uart0_filestream, &data[bytesReceived], 1);
			int errval = errno;
			if(count != 1){
				if(errval == 11){
					//"Resource temporarily unavailable" 
					//serial port not ready; spin while loop
				}else if (errval == 0){
					//"Success" -> No data was available
				}else{
					//another error
					printf("serial: serial receive error(%i = %s)\n",errval,strerror(errval));
					timeout = true;
				}
			}else{
				if(data[bytesReceived] == '\n'){
					newline = true;
				}
				if(bytesReceived > 1000){
					newline = true;
				}
				bytesReceived++;
			}
			if(GetTickCount() - startTime > READTIMEOUT){
				printf("Serial read newline timed out\n");
				timeout = true;
			}
		}
	}
	return bytesReceived;
}

/* ----------------------------------------------------------------------------
	Function: bool Serial::available()
	
	Description: Checks if there are any bytes to read from the serial buffer.
   
	Parameters: None			
	Returns: True if there are bytes to read, false otherwise.
   ------------------------------------------------------------------------*/
bool Serial::available(){
	bool ret = false;
	if (uart0_filestream != -1){
		int bytes_available = 0;
		ioctl(uart0_filestream, FIONREAD, &bytes_available);
		int errval = errno;
		if(bytes_available > 0){
			//printf("Serial: Number of bytes to be read: %i\n", bytes_available);
			ret = true;
		}else{
			if (errval == 0){
				//"Success" -> No error occured
			}else{
				//another error
				printf("Serial: serial receive error(%i = %s)\n",errval,strerror(errval));
			}
		}
	}
	return ret;
}

/* ----------------------------------------------------------------------------
	Function: void Serial::end()
	
	Description: Closes the serial connection
   
	Parameters: None					
	Returns: Nothing
   ------------------------------------------------------------------------*/
void Serial::end(){
	close(uart0_filestream);
}

/* ----------------------------------------------------------------------------
	Function: bool unitTestSerial()
	
	Description: Unit test for the serial class. 
   
	Parameters: None					
	Returns: True if test was successful, False otherwise.
	
	Notes: The GPIO serial port on the Raspberry Pi must be connected in 
		   loopback (TX<->RX) in order to test the serial class. Tx is on pin 8 
		   and RX on pin 10 for Raspberry Pi models A and B.
   ------------------------------------------------------------------------*/
bool unitTestSerial(){
	//Connect TX to RX on the serial port before testing
	printf("----------Serial Unit Test-----------\n");
	bool success = true;
	const uint16_t MAX_LEN = 2048;
	//Open server port
	std::string gpio_serial_port = "/dev/ttyAMA0";
	unsigned int baud = 230400;
	Serial serial(gpio_serial_port,baud);
	
	for(int loop = 0; loop < 10; loop++){
		//determine random length
		uint8_t testBuf[MAX_LEN];
		srand(GetTickCount());
		int len = (((uint16_t)rand()) % (MAX_LEN - 1)) + 1; //from 1 to MAX_LEN 
		
		//generate random data up to len
		for (int i = 0; i < len ; i++){
			testBuf[i]  = (uint8_t)((uint16_t)rand() % 256);
		}
		
		//Test all the functions
		//send the data
		int numSentBytes = serial.send(testBuf, len);
		if(numSentBytes != len){
			printf("Error number of sent bytes (%i) does not match length of buffer (%i)\n",numSentBytes,len);
			success = false;
		}else{
		
			
			//receive the data
			uint8_t rBuf[MAX_LEN];
			int numRecBytes = serial.get(rBuf,len);
			if(numRecBytes != len){
				printf("Error number of received bytes (%i) does not match number sent bytes (%i)\n",numRecBytes,numSentBytes);
				success = false;
			}else{
				//compare the results
				if(memcmp( &testBuf[0], &rBuf[0], numRecBytes) != 0){
					printf("Error received data does not match sent data (%i)\n",numRecBytes);
					success = false;
				}else{
					//successful read/write
					printf("(%i)Successful read; data in (%i) == data out (%i)\n",loop,len,numRecBytes);
				}
			}
		}

	}
	serial.end();
	if(success == true){
		printf("Serial unit test successful\n");
	}else{
		printf("Serial Server Unit test failed\n");
	}	
	printf("-----------------------------------\n");
	return success;
}