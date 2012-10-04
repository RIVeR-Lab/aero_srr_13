//============================================================================
// Name        : BatteryManager.cpp
// Author      : Jon Anderson jonjake
// Version     : 1.0
// Date		   : Dec 1, 2011
// Copyright   : Worcester Polytechnic Institute
// Project	   : Oryx 2.0 (www.wpirover.com)
// Description :
//============================================================================

#include <iostream>
#include <string>
#include <sstream>

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "ros/ros.h"
#include "OryxMessages/Battery.h"
#include "OryxMessages/Temperature.h"
using namespace std;

#define FALSE 0
#define TRUE  1

#define RESOLUTION 4096.0
#define DEG_PER_REV 360.0

// Default port. Assumes one BMS plugged in
// and no other USB devices posing as serial
// devices.
#define SERIAL_PORT "/dev/ttyUSB0"

// The number below is size of the longest possible response
// received from the QSB when all terminators are on and spaces
// separate the elements. It also allows for the string termination.
const int BUFFER_SIZE = 100;

// The USB link between the QSB and the computer introduces
// considerable lag. So, we need to wait a bit when sending
// an instruction before we get the response. The value of
// 5 ms is empirical and may need adjustment on other systems.
const int FIVE_MILLISECONDS = 50000;

// How many times we will try a non-blocking read of the
// device before we call it a failed operation.
const int MAX_READ_TRIES = 20;

// Rather than getting into complications reading the
// keyboard for input, we will just trap Ctrl+C and
// notify the app when the user is done.
int CloseRequested = FALSE;

int bms;

void printBMSError(char* errorMessage)
{
        printf("%s: %d - %s", errorMessage, errno, strerror(errno));
        exit(-1);
}

// The QSB presents itself to the system as a vanilla
// UART and we can talk to it using the standard
// POSIX IO functions.
//
// In the two utilitarian functions below, 'qsb' is
// a standard UNIX file handle pointing to the serial
// port where the devices is hosted.


void sendBMSInstruction(int bms, char* command)
{
    // Create a padded instruction string that includes
    // CR+LF. The QSB will be happy with just CR or LF too.
    char* bmsCommand = (char *)malloc(strlen(command) + 1);
    sprintf(bmsCommand, "%s\r", command);

    int ioResult = write(bms, bmsCommand, strlen(bmsCommand));
    free(bmsCommand);

    if (ioResult < 0)
    {
        printBMSError("Error writing to BMS device");
    }
}


void readBMSResponse(int bms, char* response, int responseSize)
{
    int i = 0;
    int ioResult;
    usleep(FIVE_MILLISECONDS);
    read(bms, response, responseSize);
    usleep(FIVE_MILLISECONDS);

    if (i == MAX_READ_TRIES)
    {
        printBMSError("Error reading from BMS device - Max Tries");
    }

    // Remove the trailing CR+LF if any, and trim to proper size.
    int end = strcspn(response, "\r\n");
    response[end] = '\0';
}


// Every instruction sent to the QSB is acknowledged
// with a corresponding response string. We then send
// the instruction and retrieve the response as a
// single command transaction.
void bmsCommand(int bms, char* command, char* response, int responseSize)
{
    sendBMSInstruction(bms, command);
    readBMSResponse(bms, response, responseSize);
}



void ctrlCHandler(int signal)
{
	 close(bms);
	 exit(1);
}


float getVoltage(int node){
	char voltage[BUFFER_SIZE];
	string voltString;
	switch (node){
	case 0:
		bmsCommand(bms, "01querytot", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 1:
		bmsCommand(bms, "01voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 2:
		bmsCommand(bms, "02voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 3:
		bmsCommand(bms, "03voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 4:
		bmsCommand(bms, "04voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 5:
		bmsCommand(bms, "05voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 6:
		bmsCommand(bms, "06voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	case 7:
		bmsCommand(bms, "07voltage", voltage, BUFFER_SIZE);
		voltString=voltage;
		voltString.erase(0,4);
		return strtod(voltString.c_str(),NULL);
		break;
	default:
		break;
	}
}

float getTemperature(int node){
	char temperature[BUFFER_SIZE];
	string temperatureString;
	switch (node){
	case 0:
		bmsCommand(bms, "01temperat", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	case 1:
		bmsCommand(bms, "01xtrntemp", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	case 2:
		bmsCommand(bms, "02xtrntemp", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	case 3:
		bmsCommand(bms, "03xtrntemp", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	case 4:
		bmsCommand(bms, "04xtrntemp", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	case 5:
		bmsCommand(bms, "05xtrntemp", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	case 6:
		bmsCommand(bms, "06xtrntemp", temperature, BUFFER_SIZE);
		temperatureString=temperature;
		temperatureString.erase(0,4);
		return strtod(temperatureString.c_str(),NULL);
		break;
	default:
		break;
	}
}

void configureBMS(std::string port){
	// Open the port.  If a port is passed as parameter use that otherwise use the default one /dev/ttyUSB0

	    for( int i=0; i<50;i++){
	    	bms = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	    	if(bms>=0) break;
	    	usleep(100000);
			ROS_ERROR_STREAM("Could Not Open BMS. Retrying...");
	    }
	    if (bms < 0)
	    {
	        ROS_ERROR_STREAM("Error opening BMS device");
	        exit(0);
	    }


	    // Configure it.
	    struct termios bmsConfiguration;
	    tcgetattr(bms, &bmsConfiguration);

	    // Baud 9600 and 8 Bit, 1 Stop Bit, No-Parity
	    bmsConfiguration.c_cflag = B9600 | CS8;
	    //fcntl(bms, F_SETFL, 200);
	    cfsetospeed(&bmsConfiguration, B9600);
	    // Flush immediately
	    tcflush(bms, TCIFLUSH);
	    // Change attributes now
	    tcsetattr(bms, TCSAFLUSH, &bmsConfiguration);

	    // Below is the basic communications protocol with
	    // the QSB. The commands were composed using the
	    // QSB command list available for download at the
	    // US Digital site.

	    char v_t[BUFFER_SIZE], v_1[BUFFER_SIZE], v_2[BUFFER_SIZE], v_3[BUFFER_SIZE],
			v_4[BUFFER_SIZE], v_5[BUFFER_SIZE], v_6[BUFFER_SIZE];
	    char temp_t[BUFFER_SIZE], temp_1[BUFFER_SIZE], temp_2[BUFFER_SIZE],
			temp_3[BUFFER_SIZE], temp_4[BUFFER_SIZE], temp_5[BUFFER_SIZE],
			temp_6[BUFFER_SIZE];
	    return;
}

// *************************************************
//
//  General flow of operation:
//   1. Open the port and configure it.
//      The QSB UART wants 230.4K Baud, 8-n-1
//   2. Set up the QSB to read an encoder.
//   3. Loop polling and printing the current
//      encoder count.
//   4. Upon close request, close the port and
//      exit.
//
// *************************************************

int main (int argc, char *argv[])
{
	double danger_Temp = 150;
	double warning_temp = 130;
	double loop = .25;
    std::string port = SERIAL_PORT;
    // Register the Ctrl-C handler.
    signal(SIGINT, ctrlCHandler);

    ros::init(argc, argv, "BatteryManager");
    ros::NodeHandle n;

    ros::param::get("~Danger_Temp",danger_Temp);
    ros::param::get("~Warning_Temp",warning_temp);
    ros::param::get("~Loop_Rate",loop);
    ros::param::get("~Port",port);

    ros::Publisher batteryPublisher = n.advertise<OryxMessages::Battery>("Battery",6);
    ros::Publisher temperaturePublisher = n.advertise<OryxMessages::Temperature>("Temps",15);

    ros::Rate loopRate(loop);

    configureBMS(port);

    while (n.ok())
       {


    	OryxMessages::Battery batteryMsg;
    	OryxMessages::Temperature temperatureMsg;

    	temperatureMsg.warning_Temp = warning_temp;
    	temperatureMsg.danger_Temp = danger_Temp;

		batteryMsg.node=0;
		batteryMsg.voltage=getVoltage(0);
		temperatureMsg.temperature_node=17;
		temperatureMsg.temperature=getTemperature(0);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=1;
		batteryMsg.voltage=getVoltage(1);
		temperatureMsg.temperature_node=18;
		temperatureMsg.temperature=getTemperature(1);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=2;
		batteryMsg.voltage=getVoltage(2);
		temperatureMsg.temperature_node=19;
		temperatureMsg.temperature=getTemperature(2);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=3;
		batteryMsg.voltage=getVoltage(3);
		temperatureMsg.temperature_node=20;
		temperatureMsg.temperature=getTemperature(3);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=4;
		batteryMsg.voltage=getVoltage(4);
		temperatureMsg.temperature_node=21;
		temperatureMsg.temperature=getTemperature(4);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=5;
		batteryMsg.voltage=getVoltage(5);
		temperatureMsg.temperature_node=22;
		temperatureMsg.temperature=getTemperature(5);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=6;
		batteryMsg.voltage=getVoltage(6);
		temperatureMsg.temperature_node=23;
		temperatureMsg.temperature=getTemperature(6);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);

		batteryMsg.node=7;
		batteryMsg.voltage=getVoltage(7);
		if(batteryMsg.voltage >0)batteryPublisher.publish(batteryMsg);
		if(temperatureMsg.temperature > 0) temperaturePublisher.publish(temperatureMsg);
		ros::spinOnce();

		loopRate.sleep();

       }

    close(bms);
    //puts("\n");
    return(0);
}
