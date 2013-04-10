/*
 * OutputMotorToSerial.cpp
 *
 * Nathan Grubb, March 2013
 * Serial Read/Write code derived from 
 *     http://www.tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
 *
 * This ROS node subscribes to messages from other nodes that wish to drive the motors.
 * It does some safety checks, then outputs the drive signals to an arduino which will
 * drive the motor PWM. 
 *
 */

// serial
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h>

// CPP
#include <sstream>
#include "OutputMotorToSerial.h"

#define BAUDRATE B9600            
#define _POSIX_SOURCE 1 


OutputMotorToSerial::OutputMotorToSerial( ){ }

void OutputMotorToSerial::callbackBase( const boost::shared_ptr<alfred_msg::DriveBase const>& driveOrder){

   ROS_INFO( "Received Base Order: [%f,%f,%f]", driveOrder->motor1, driveOrder->motor2, driveOrder->motor3);

}

void OutputMotorToSerial::start(int argc, char** argv){

   /********************
    * Set up ROS node
    */
   ros::init(argc, argv, "talker");
   ros::NodeHandle n;

   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = n.subscribe("DriveBase", 2, OutputMotorToSerial::staticCallbackBase ); 

   ROS_INFO( "Starting serial" );
   
   /*******************
    * Set up serial writer
    */
   int fd;
   struct termios oldtio,newtio;
   char buf[255];

   char* modemDevice;
   mSerialDevice =  "/dev/ttyUSB0";
   if( argc > 1 )
     mSerialDevice = argv[1];

   fd = open(modemDevice, O_RDWR | O_NOCTTY | O_NDELAY);
   if (fd <0) {perror(modemDevice); return ; }
         
   tcgetattr(fd,&oldtio); /* save current serial port settings */
   memset(&newtio, 0, sizeof newtio );
         
   /* 
      BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
      CRTSCTS : output hardware flow control (only used if the cable has
      all necessary lines. See sect. 7 of Serial-HOWTO)
      CS8     : 8n1 (8bit,no parity,1 stopbit)
      CLOCAL  : local connection, no modem contol
      CREAD   : enable receiving characters
   */
   newtio.c_cflag = BAUDRATE /*| CRTSCTS*/ | CS8 | CLOCAL /*| CREAD*/;
          
   /*
     IGNPAR  : ignore bytes with parity errors
     ICRNL   : map CR to NL (otherwise a CR input on the other computer
     will not terminate input)
     otherwise make device raw (no other input processing)
   */
   //newtio.c_iflag = IGNPAR | ICRNL;
          
   /*
     Raw output.
   */
 //  newtio.c_oflag = 0;
          
   /*
     ICANON  : enable canonical input
     disable all echo functionality, and don't send signals to calling program
   */
 //  newtio.c_lflag = ICANON;

 #if 0    
   // TODO - do we need this control characters crap?
   /* 
      initialize all control characters 
      default values can be found in /usr/include/termios.h, and are given
      in the comments, but we don't need them here
   */
   newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
   newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
   newtio.c_cc[VERASE]   = 0;     /* del */
   newtio.c_cc[VKILL]    = 0;     /* @ */
   newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
   newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
   newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
   newtio.c_cc[VSWTC]    = 0;     /* '\0' */
   newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
   newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
   newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
   newtio.c_cc[VEOL]     = 0;     /* '\0' */
   newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
   newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
   newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
   newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
   newtio.c_cc[VEOL2]    = 0;     /* '\0' */
         
 #endif  
   /* 
      now clean the modem line and activate the settings for the port
   */
   tcflush(fd, TCIFLUSH);
   tcsetattr(fd,TCSANOW,&newtio);

   /*******************
    * now loop, write serial and publish messages
    */
   const std::string whiteSpaces( " \f\n\r\t\v" );
   int index(0);
   ros::Rate rate(300);

   while (1) {
       //write
       index++;
       char writeBuf[255];
       if( index % 2 == 0 )
          strcpy(writeBuf, "1\r\n");
       else
          strcpy(writeBuf, "0\r\n");
       //buf[1]='\0'; 
       int sizeWritten = write(fd,writeBuf,3);
       rate.sleep();
   }
   /* restore the old port settings */
   tcsetattr(fd,TCSANOW,&oldtio);
   close(fd);
}

int main(int argc, char** argv)
{
  gOutputMotorHandler = new OutputMotorToSerial();
  gOutputMotorHandler->start(argc, argv);

  return 0;
}
