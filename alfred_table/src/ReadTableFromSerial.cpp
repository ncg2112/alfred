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

#if 0
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
#endif
//serial
#include "alfred_msg/SerialIO.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "alfred_msg/FSRDirection.h"
#include "alfred_msg/FSRUpDown.h"

// CPP
#include <sstream>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#define BAUDRATE B115200


#if 0
struct termios oldtio;

int serialport_init(const char* serialport, int baud);
void serialport_close(int fd);
int serialport_write(int fd, const char* msg);
int serialport_read_until(int fd, char* buf, char until);

#endif

int main(int argc, char** argv)
{
  /********************
   * Set up ROS node
   */
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher tableDir_pub = n.advertise<alfred_msg::FSRDirection>("table_direction", 100);
  ros::Publisher tableUpDown_pub = n.advertise<alfred_msg::FSRUpDown>("table_updown", 100);
  alfred_msg::FSRDirection directionMessage;

  // Set up serial connection
  char buf[20];
  char* modemDevice =  "/dev/ttyACM0";
  if( argc > 1 )
    modemDevice = argv[1];
  int fd = serialport_init(modemDevice, BAUDRATE);  
  if(fd < 0)  return -1;
  usleep(3000 * 1000 ); 
   
  printf("Connected to Arduino, starting loop\n");
  ros::Rate rate(50);
  while(ros::ok())
  {
      // Get the X coordinate
      serialport_write(fd, "x\0");
      serialport_read_until(fd, buf, ':');
      directionMessage.x = atof(buf);
      //printf("Received X:%f\n", directionMessage.x); 
      
      // Get the Y coordinate
      serialport_write(fd, "y");
      serialport_read_until(fd, buf, ':');
      directionMessage.y = atof(buf);
      //printf("Received Y:%f\n", directionMessage.y); 

      // Get the follow indicator
      serialport_write(fd, "f");
      serialport_read_until(fd, buf, ':');
      directionMessage.followPressed = atof(buf) == 1;
      //printf("Received F:%s\n", directionMessage.followPressed ? "true":"false" );

      tableDir_pub.publish(directionMessage);
      rate.sleep();
  }
}

#if 0

//===========================================
// SERIAL COMMUNICATION FUNCTIONS
//===========================================
void serialport_close(int fd)
{
   // restore the old port settings
   tcsetattr(fd,TCSANOW,&oldtio);
   close(fd);
}

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
    //        serialport,baud);
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NOCTTY);
    if (fd == -1)  
    {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    if (tcgetattr(fd, &toptions) < 0) 
    {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    tcgetattr(fd, &oldtio); // save current serial port setting
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) 
    {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
      // if you want these speeds, uncomment these and set #defines if Linux
      //#ifndef OSNAME_LINUX
      //    case 14400:  brate=B14400;  break;
      //#endif
    case 19200:  brate=B19200;  break;
      //#ifndef OSNAME_LINUX
      //    case 28800:  brate=B28800;  break;
      //#endif
      //case 28800:  brate=B28800;  break;
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;

    if( tcsetattr(fd, TCSANOW, &toptions) < 0) 
    {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }
 
    return fd;
}


int serialport_read_until(int fd, char* buf, char until)

{
    char b[1];
    int i=0;
    do 
    { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) 
	     {
            usleep( 10 * 1000 ); // wait 10 msec try again
            continue;
        }
        buf[i] = b[0]; 
	 i++;
    } while( b[0] != until );
    buf[i-1] = 0;  // null terminate the string
    return i-1;
}

int serialport_write(int fd, const char* msg)
{
    char send[255];
    int n = sprintf( send, msg );
    int written = write( fd, send, n );
    //printf( "I wrote %s (%i) \t", msg, written);
    return written;
}

#endif
