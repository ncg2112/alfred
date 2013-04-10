/*
 * ReadTableFromSerial.cpp
 *
 * Nathan Grubb, March 2013
 * Serial Read/Write code derived from 
 *     http://www.tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
 *
 * This ROS node reads the USB serial port data coming from the sensor
 * pre-proccessor board.
 * It does some formatting and sanity checks, and publishes them as ROS topics
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

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alfred_msg/FSRDirection.h"
#include "alfred_msg/FSRUpDown.h"

// CPP
#include <sstream>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#define BAUDRATE B9600            
#define _POSIX_SOURCE 1 

int main(int argc, char** argv)
{
  /********************
   * Set up ROS node
   */
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher tableDir_pub = n.advertise<alfred_msg::FSRDirection>("table_direction", 100);
  ros::Publisher tableUpDown_pub = n.advertise<alfred_msg::FSRUpDown>("table_updown", 100);

  /*******************
   * Set up serial reader
   */
  int fd,c, res;
  struct termios oldtio,newtio;
  char buf[255];

  char* modemDevice;
  modemDevice =  "/dev/ttyUSB0";
  if( argc > 1 )
    modemDevice = argv[1];

  fd = open(modemDevice, O_RDWR | O_NOCTTY ); 
  if (fd <0) {perror(modemDevice); return -1; }
        
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
  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
         
  /*
    IGNPAR  : ignore bytes with parity errors
    ICRNL   : map CR to NL (otherwise a CR input on the other computer
    will not terminate input)
    otherwise make device raw (no other input processing)
  */
  newtio.c_iflag = IGNPAR | ICRNL;
         
  /*
    Raw output.
  */
  newtio.c_oflag = 0;
         
  /*
    ICANON  : enable canonical input
    disable all echo functionality, and don't send signals to calling program
  */
  newtio.c_lflag = ICANON;
    
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
        
  /* 
     now clean the modem line and activate the settings for the port
  */
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
  
  /*******************
   * now loop, read serail and publish messages
   */
  const std::string whiteSpaces( " \f\n\r\t\v" );
  while (ros::ok()) {
    res = read(fd,buf,255); 
    buf[res]=0;  /* set end of string, so we can printf */
    std::string readIn(buf);
    std::string::size_type pos = readIn.find_last_not_of( whiteSpaces );
    readIn.erase( pos + 1 );  

    std_msgs::String msg;
    msg.data = readIn;
    if( readIn.compare( "" ) ){
      ROS_INFO( "Recieved [%s]\n", readIn.c_str() );

      boost::regex e("^DirX: ([-+]?[0-9]*\.?[0-9]+);DirY: ([-+]?[0-9]*\.?[0-9]+);Follow: ([01]);UpDown ([-]?[01]);");
      boost::smatch result;
      if( ! boost::regex_search(readIn, result, e) ){
	ROS_ERROR( "ERROR! Message recieved over UART wrong format!\n");
	continue;
      }

      //double dirY = boost::lexical_cast<double>(result[1].first);
      ROS_INFO( "DirY is %f\n", dirY );

      tableDir_pub.publish(msg);
      tableUpDown_pub.publish(msg);
    }
  }
  /* restore the old port settings */
  tcsetattr(fd,TCSANOW,&oldtio);
}
