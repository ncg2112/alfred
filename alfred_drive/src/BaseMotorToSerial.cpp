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

// ROS
#include "ros/ros.h"
#include "alfred_msg/DriveBase.h"

// serial
#include "alfred_msg/SerialIO.h"

// CPP
#include <string>
#include <sstream>

#define _POSIX_SOURCE 1 

/*
 * Global state vars for serial output
 * NOTE: These names are long because they're global
 */
double baseOrder_serialArduino1;
double baseOrder_serialArduino2;
double baseOrder_serialArduino3;

bool receivedOrder_serialArduino;

void callbackBaseMotorToSerial( const boost::shared_ptr<alfred_msg::DriveBase const>& driveOrder){

   ROS_DEBUG( "Received Base Order: [%f,%f,%f]", driveOrder->motor1, driveOrder->motor2, driveOrder->motor3);
   
   baseOrder_serialArduino1 = driveOrder->motor1;
   baseOrder_serialArduino2 = driveOrder->motor2;
   baseOrder_serialArduino3 = driveOrder->motor3;
   
   receivedOrder_serialArduino = true;
}

int main(int argc, char** argv){

   /*******************
    *  Base Drive Variables
    *
    * LOOP_RATE         defines how fast we send orders to the arduino
    * ORDER_TIMEOUT     is the maximum amount of time between recieving orders
    *                   before we shut the motors off
    *
    * BAUDRATE          baudrate to arduino
    * SERIAL_DEV        the defauly dev file pointing to the arduino
    */
   double LOOP_RATE = 20.0;
   double ORDER_TIMEOUT = 0.25;

   int BAUDRATE = B115200;
   char* SERIAL_DEV = "/dev/ttyUSB0";

   /********************
    * Set up ROS node
    */
   ros::init(argc, argv, "BaseMotorToSerial");
   ros::NodeHandle n;

   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = n.subscribe("DriveBase", 2, callbackBaseMotorToSerial ); 

   /********************
    * Setup loop variables and start serial
    */
   char* serialDevice = SERIAL_DEV;
   if( argc > 1 )
      serialDevice = argv[1];

   int fd = serialport_init(serialDevice, BAUDRATE);
   if(fd < 0){
      ROS_ERROR( "Could not open serial device \'%s\'\n", serialDevice); 
      return -1;
   }
   usleep( 3000 * 1000 );
   
   ros::Rate rate(LOOP_RATE);
   int maxNumLoopsWithoutOrder = (int)( ORDER_TIMEOUT * LOOP_RATE );
   int numLoopsWithoutOrder(-1);
    
   
   while (n.ok()) {

      // If we haven't yet started
      if( numLoopsWithoutOrder == -1){
         baseOrder_serialArduino1 = 40.0;
         baseOrder_serialArduino2 = 0.0;
         baseOrder_serialArduino3 = 0.0;
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){

         baseOrder_serialArduino1 = 0.0;
         baseOrder_serialArduino2 = 0.0;
         baseOrder_serialArduino3 = 0.0;
         ROS_ERROR( "Base Motor to Serial hasn't received a new motor order in %f mS!", 1000 * ORDER_TIMEOUT );
      }
      // Safety check passed, send 
    
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( receivedOrder_serialArduino == true ){
         numLoopsWithoutOrder = 0;
         receivedOrder_serialArduino = false;
      }

      ROS_INFO( "Sending output to base: [%f, %f, %f]",
                  baseOrder_serialArduino1,
                  baseOrder_serialArduino2,
                  baseOrder_serialArduino3);
 
      //write
      std::stringstream ss;
      ss << baseOrder_serialArduino1 << ";"
         << baseOrder_serialArduino2 << ";"
         << baseOrder_serialArduino3 << ";\0";
      std::string msg = ss.str();
   
      ROS_INFO("Sending over serial\'%s\'", msg.c_str() );

      int isok = serialport_write(fd, msg.c_str() );
      
      ROS_INFO("send returned %d", isok);
   
      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }

   serialport_close(fd);
   return 0;
}




