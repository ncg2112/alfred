/*
 * SerialIO.h
 *
 * Joe DelPreto & Nathan Grubb, March 2013
 * Serial Read/Write code derived from 
 *   Tod E. Kurt, tod@todbot.com
 *
 * A header file containing useful functions for serial over USB
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <map>

/*
 * Function Definitions
 */

int serialport_init(const char* serialport, int baud);
int serialport_writebyte(int fd, uint8_t b);
int serialport_write(int fd, const char* str);
int serialport_read_until(int fd, char* buf, char until);

/*
 * A map to hold all of the old termio settings,
 * with the fd as key
 */ 
std::map<int,termios> serialport_oldtermios;

void serialport_close(int fd)
{
   //restore old port settings
   tcsetattr(fd, TCSANOW, &serialport_oldtermios[fd]);
   close(fd);
}

int serialport_writebyte( int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return -1;
    return 0;
}


int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if( n!=len ) 
        return -1;
    return n;
}


/*
 * Reads from the serial port indicaed by fd till the char until
 * is hit, then copies that into buf;
 *
 * @param  int    fd
 * @arg    char*  buf. Valid if return != -1
 * @param  char   until
 * @param  int    timeout in us
 *
 * @return int   the number of characters read. -1 if failed
 */
int serialport_read_until(int fd, char* buf, char until, int timeout)
{
    // delay to wait if we don't see a char immediately
    int uSecDelay(10);
    timeout /= uSecDelay;  // convert to # loops
    char b[1];
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            //usleep( 10 ); // wait 10 usec try again
            continue;
        }
        buf[i] = b[0]; i++;
    } while( b[0] != until && timeout-- > 0);
    if( timeout <= 0 )
      return -1;

    buf[i] = '\0';  // null terminate the string
    return i;
}


/* takes the string name of the serial port (e.g. "/dev/ttyUSB0","COM1")
 * and a baud rate (bps) and connects to that port at that speed and 8N1.
 * opens the port in fully raw mode so you can send binary data.
 * returns valid fd, or -1 on error
 */

int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NOCTTY);

    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }

    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    struct termios oldtio;
    tcgetattr(fd, &oldtio);
    serialport_oldtermios[fd] = oldtio;

    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
    case 19200:  brate=B19200;  break;
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
  
    // turn on READ & ignore ctrl lines
    toptions.c_cflag |= CREAD | CLOCAL;

    // turn off s/w flow ctrl
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); 

    // make raw
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 

    // make raw
    toptions.c_oflag &= ~OPOST;



    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;

    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}
