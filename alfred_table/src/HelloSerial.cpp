#include "alfred_msg/SerialIO.h"

int main(int argc, char *argv[]) 

{

    int fd = 0;

    char serialport[256];

    int baudrate = B115200;  // default

    char buf[20], dat[20], use[3];

    int rc,n;



      //baudrate = 9600;

      fd = serialport_init("/dev/ttyACM0", baudrate);

                  if(fd==-1) return -1;

      usleep(3000 * 1000 );

            

      while(1) {

            strcpy(dat, "00000000:\0");

            gets(use);

            if(use[0] == 'f') {

                  dat[0] = 'f';

                  dat[1] = 5;

            }

            else if(use[0] == 'b') {

                  dat[0] = 'b';

                  dat[1] = 5;

            }

            else if(use[0] == 'j') {

                  dat[2] = 'f';

                  dat[3] = 5;

            }

            else if(use[0] == 'l') {

                  dat[2] = 'b';

                  dat[3] = 5;

            }

                  

            rc = serialport_write(fd, dat);

                        if(rc==-1) return -1;

             //printf("Waiting until UART buffer clears: %d\n", tcdrain(fd));

             n = serialport_read_until(fd, buf, ':');

               printf("wrote %d bytes, read %d bytes: %s\n", rc, n, buf);

      }



       close(fd);



    exit(EXIT_SUCCESS);    

} // end main

