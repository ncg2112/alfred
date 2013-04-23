/*
 * BaseMotorToSerial.cpp
 *
 * Nathan Grubb, March 2013
 *
 * This ROS node subscribes to messages from other nodes that wish to drive the motors.
 * It does some safety checks, then outputs the drive signals to an arduino which will
 * drive the motor PWM. 
 *
 */

// serial
#include "alfred_msg/SerialIO.h"

// CPP
#include <sstream>
#include "BaseMotorToSerial.h"

#define BAUDRATE B9600            
#define _POSIX_SOURCE 1 


BaseMotorToSerial::BaseMotorToSerial( ){ 
   
   mTargetBase1 = .5;
   mTargetBase2 = .5;
   mTargetBase3 = .5;

   mBase1 = .5;
   mBase2 = .5;
   mBase3 = .5;
}

void BaseMotorToSerial::callbackBase( const boost::shared_ptr<alfred_msg::DriveBase const>& driveOrder){

   ROS_DEBUG( "Received Base Order: [%f,%f,%f]", driveOrder->motor1, driveOrder->motor2, driveOrder->motor3);
   
   mTargetBase1 = driveOrder->motor1;
   mTargetBase2 = driveOrder->motor2;
   mTargetBase3 = driveOrder->motor3;
   
   mReceivedOrder = true;
}

void BaseMotorToSerial::start(int argc, char** argv){

   /********************
    * Set up ROS node
    */
   ros::init(argc, argv, "talker");
   ros::NodeHandle n;

   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = n.subscribe("DriveBase", 2, BaseMotorToSerial::staticCallbackBase ); 

   /********************
    * Setup loop variables and start serial
    */
   if( argc > 1 )
      mSerialDevice = argv[1];
   else
      mSerialDevice = "/dev/ttyACM0";

   int fd = serialport_init(mSerialDevice.c_str(), B115200);
   if(fd < 0){
      ROS_ERROR( "Could not open serial device \'%s\'\n", mSerialDevice.c_str() ); 
      return;
   }
   usleep( 3000 * 1000 );
   
   double loopRate = 10.0;     // In Hz
   ros::Rate rate(loopRate);

   double orderTimeout = 0.25;    // In seconds
   int maxNumLoopsWithoutOrder = (int)( orderTimeout * loopRate );
   int numLoopsWithoutOrder(-1);
    
   double maxAcc = 0.1;     // measured in % per second
   double maxAccPerLoop = maxAcc / loopRate;

   ROS_DEBUG( "Max Acc Per Loop: %f", maxAccPerLoop );

   while (n.ok()) {

      // If we haven't yet started
      if( numLoopsWithoutOrder == -1){
         mBase1 = .5;
         mBase2 = .5;
         mBase3 = .5;
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){

         mBase1 = .5;
         mBase2 = .5;
         mBase3 = .5;
         ROS_ERROR( "Base Motor to Serial hasn't received a new motor order in %f mS!", 1000 * orderTimeout );
      }
      // Safety check passed, ramp motors to target
      else{

         mBase1 = mTargetBase1;
         mBase2 = mTargetBase2;
         mBase3 = mTargetBase3;

      }
    
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( mReceivedOrder == true ){
         numLoopsWithoutOrder = 0;
         mReceivedOrder = false;
      }

      ROS_INFO( "Sending output to base: [%f, %f, %f]", mBase1, mBase2, mBase3);
 
      //write
      std::stringstream ss;
      ss << mBase1 << ";" << mBase2 << ";" << mBase3 << ";\0";
      std::string msg = ss.str();

      serialport_write(fd, msg.c_str() );

      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }

   serialport_close(fd);
}

int main(int argc, char** argv)
{
  gBaseMotorHandler = new BaseMotorToSerial();
  gBaseMotorHandler->start(argc, argv);

  return 0;
}



/****************************************************
OLD SERIAL
*/
#if 0 
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

   int loopRate = 100;           // in Hz
   ros::Rate rate(loopRate);
   /*
    * Do safety checks in this loop.
    * First Order: If no orders received within the last 250 ms, turn off motors
    * Second Order: TODO
    */
#endif


