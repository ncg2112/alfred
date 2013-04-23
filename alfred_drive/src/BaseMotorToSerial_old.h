/*
 * BaseMotorToSerial.h
 *
 * Nathan Grubb, March 2013
 * Serial Read/Write code derived from 
 *     http://www.tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
 * 
 * The header file for class BaseMotorToSerial
 * This ROS node subscribes to messages from other nodes that wish to drive the motors.
 * It does some safety checks, then outputs the drive signals to an arduino which will
 * drive the motor PWM.
 */ 

// ROS
#include "ros/ros.h"
#include "alfred_msg/DriveBase.h"
#include "alfred_msg/FSRDirection.h"
#include "alfred_msg/FSRUpDown.h"

// cpp
#include <string>


class BaseMotorToSerial {

 public:
  
  /*
   * Constructor
   */
   BaseMotorToSerial( );

  /* 
   * Starts the subscribe/write to serial loop
   */
   void start( int, char** );
  
  /*
   * Callback function when topics are published
   */ 
   void callbackBase( const boost::shared_ptr<alfred_msg::DriveBase const>& );

  /*
   * Static function wrapper for callbackBase, using the global variable below
   */
   static void staticCallbackBase( const boost::shared_ptr<alfred_msg::DriveBase const>& msg);

 private:
  
  /*
   * The target speeds and current speeds, as percents
   * (These are different so that we can ramp up/down)
   */
   double mTargetBase1;
   double mTargetBase2;
   double mTargetBase3;

   double mBase1;
   double mBase2;
   double mBase3;
 
   bool mReceivedOrder;

   /*
    * The name of the serial device in /dev
    */
   std::string mSerialDevice;
      
}; 

/*
 * Global variable, neccessary because the ROS subscribe requires a static function pointer
 * and definiton of static wrapper
 */
BaseMotorToSerial* gBaseMotorHandler;

void BaseMotorToSerial::staticCallbackBase( const boost::shared_ptr<alfred_msg::DriveBase const>& msg){
   gBaseMotorHandler->callbackBase(msg);
}
