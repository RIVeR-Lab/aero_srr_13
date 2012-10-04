//============================================================================
// Name        : qsb_encoder.cpp
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
#include "std_msgs/Float32.h"

using namespace std;

#define FALSE 0
#define TRUE  1

#define RESOLUTION 4096.0
#define DEG_PER_REV 360.0

// Default port. Assumes one QSB plugged in
// and no other USB devices posing as serial
// devices.
#define SERIAL_PORT "/dev/rocker"



// The response from a QSB has the general format:
//
//     Element: Response  Register  Data  TimeStamp  EOR
//     Bytes  :     1         2       8       8       4
//
//
// Response, Register and Data, are always present.
//
// The rest of the elements are determined by the EOR (15) register.
// Each feature is enabled with a bit=1 or disabled with a bit=0.
// The default value is: CR/LF = 0x03.
//
// BITS:   B3 B2 B1 B0
//
// B0 = Line Feed
// B1 = Carriage Return
// B2 = 4-byte Time Stamp appended to the response
// B3 = Spaces between returned fields
//
// The timestamp is a 4-byte value but is appended to the
// paket as an 8-byte hex string.
//
// If spaces are used, the resulting format is:
//
//   Response[space]Register[space]Data[space]TimeStampEOR
//
// Thus the last space is right after the data value. Any
// trailing information (time stamp, etc.) is not separated
// by spaces.
//
// The number below is size of the longest possible response
// received from the QSB when all terminators are on and spaces
// separate the elements. It also allows for the string termination.
const int BUFFER_SIZE = 27;


// The USB link between the QSB and the computer introduces
// considerable lag. So, we need to wait a bit when sending
// an instruction before we get the response. The value of
// 5 ms is empirical and may need adjustment on other systems.
const int FIVE_MILLISECONDS = 5000;


// How many times we will try a non-blocking read of the
// device before we call it a failed operation.
const int MAX_READ_TRIES = 20;



// Rather than getting into complications reading the
// keyboard for input, we will just trap Ctrl+C and
// notify the app when the user is done.
int CloseRequested = FALSE;



void printQsbError(char* errorMessage)
{
       ROS_ERROR("%s: %d - %s", errorMessage, errno, strerror(errno));
       exit(-1);
}




// The QSB presents itself to the system as a vanilla
// UART and we can talk to it using the standard
// POSIX IO functions.
//
// In the two utilitarian functions below, 'qsb' is
// a standard UNIX file handle pointing to the serial
// port where the devices is hosted.


void sendQsbInstruction(int qsb, char* command)
{
    // Create a padded instruction string that includes
    // CR+LF. The QSB will be happy with just CR or LF too.
    char* qsbCommand = (char *)malloc(strlen(command) + 3);
    sprintf(qsbCommand, "%s\r\n", command);

    int ioResult = write(qsb, qsbCommand, strlen(qsbCommand));
    free(qsbCommand);

    if (ioResult < 0)
    {
        printQsbError("Error writing to QSB device");
    }
}


void readQsbResponse(int qsb, char* response, int responseSize)
{
    int i = 0;
    int ioResult;
    do
    {
        ioResult = read(qsb, response, responseSize);
        // This delay is to give some time to the device to
        // pipe the information to the serial port.
        usleep(FIVE_MILLISECONDS);
        i++;
    } while (ioResult < 0 && errno == EAGAIN && i < MAX_READ_TRIES);

    if (i == MAX_READ_TRIES)
    {
        printQsbError("Error reading from QSB device");
    }

    // Remove the trailing CR+LF if any, and trim to proper size.
    int end = strcspn(response, "\r\n");
    response[end] = '\0';

    if (ioResult < responseSize)
    {
        response[ioResult] = '\0';
    }
}


// Every instruction sent to the QSB is acknowledged
// with a corresponding response string. We then send
// the instruction and retrieve the response as a
// single command transaction.
void qsbCommand(int qsb, char* command, char* response, int responseSize)
{
    sendQsbInstruction(qsb, command);
    readQsbResponse(qsb, response, responseSize);
}



void ctrlCHandler(int signal)
{
    CloseRequested = TRUE;
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
	double offset=0;
	double rate=0;
    // Register the Ctrl-C handler.
    signal(SIGINT, ctrlCHandler);

    // Initialize ROS node and publisher
    ros::init(argc, argv, "rocker_position");

    ros::NodeHandle n;

    std::string port;
    // Open the port.  If a port is passed as argument use that otherwise use the default one /dev/ttyUSB0
    int qsb;
	port = SERIAL_PORT;
	ros::param::get("~Offset",offset);
    ros::param::get("~Port",port);
    ros::param::get("~Rate",rate);
    qsb = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    for( int i=0; i<50;i++){
    	qsb = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    	if(qsb>=0) break;
    	sleep(10);
    }
    if (qsb < 0)
    {
        printQsbError("Error opening QSB device");
    }
	

    // Configure it.
    struct termios qsbConfiguration;
    tcgetattr(qsb, &qsbConfiguration);

    qsbConfiguration.c_cflag = B230400 | CS8;
    cfsetospeed(&qsbConfiguration, B230400);
    tcflush(qsb, TCIFLUSH);
    tcsetattr(qsb, TCSANOW, &qsbConfiguration);



    // Below is the basic communications protocol with
    // the QSB. The commands were composed using the
    // QSB command list available for download at the
    // US Digital site.


    char* command;
    char response[BUFFER_SIZE];


    // Read and print product information.
    // Register: VERSION (0x14)
    qsbCommand(qsb, "R14", response, BUFFER_SIZE);
   // printf("Product info: %s\n", response);

    // Set encoder in PWM mode.
    // Register: MODE (0x00)
    qsbCommand(qsb, "W0001", response, BUFFER_SIZE);
    //printf("Mode response: %s\n", response);

    // Read encoder resolution
    // Register: MODE (0x00)
    qsbCommand(qsb, "R00", response, BUFFER_SIZE);

    char * pch;
    unsigned long resolution;
    pch = strtok(response, " ");
    pch = strtok(NULL, " ");
    pch = strtok(NULL, " ");
    resolution = strtoul (pch, NULL, 16);
    if(resolution != 0){
    	resolution = 12;
    } else resolution = 10;

    //cout << "Encoder resolution: " << resolution << "-bit" << endl;

   // puts("\nUse Ctrl+C to exit.\n\n");



    ros::Publisher rocker_position_pub = n.advertise<std_msgs::Float32>
		("Rocker_Position", 1);

    ros::Rate loop_rate(rate);


    while (CloseRequested == FALSE && ros::ok())
    {
    	unsigned long ticks;
    	float degrees;
    	std_msgs::Float32 msg;
		
        // Read current count.
        // Register: READ ENCODER (0x0E)
        qsbCommand(qsb, "R0E", response, BUFFER_SIZE);

       // printf("Current count: %s\r", response);

        pch = strtok(response, " ");
        pch = strtok(NULL, " ");
        pch = strtok(NULL, " ");
        ticks = strtoul (pch, NULL, 16);
        degrees = DEG_PER_REV * ((float) ticks / RESOLUTION)-(float)offset;

        // Publish to ROS
        msg.data = degrees;
        rocker_position_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    close(qsb);
    puts("\n");
    return(0);
}
