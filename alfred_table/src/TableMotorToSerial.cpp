/*
 * TableMotorToSerial.cpp
 *
 * Nathan Grubb, March 2013
 *
 * This ROS node subscribes to messages from other nodes that wish to drive the table motors.
 * It does some safety checks, then outputs the drive signals to an arduino which will
 * drive the motor PWM.
 * It also reads the encoder values from the arduino at the same time.
 *
 * The format for messages is:
 *  Atom Query:        `%f;%f;%f;%f;
 *  Arduino Response:  `%d;%d;%d;%d;
 */

// ROS
#include "ros/ros.h"
#include "alfred_msg/DriveTable.h"
#include "alfred_msg/TableEncoders.h"

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
double tableOrder_serialArduino1;
double tableOrder_serialArduino2;
double tableOrder_serialArduino3;
double tableOrder_serialArduino4;

bool receivedOrder_serialArduino;

void callbackTableMotorToSerial( const boost::shared_ptr<alfred_msg::DriveTable const>& driveOrder){

   ROS_DEBUG( "Received Base Order: [%f,%f,%f]", driveOrder->motor1, driveOrder->motor2, driveOrder->motor3);
   
   tableOrder_serialArduino1 = driveOrder->motor1;
   tableOrder_serialArduino2 = driveOrder->motor2;
   tableOrder_serialArduino3 = driveOrder->motor3;
   tableOrder_serialArduino4 = driveOrder->motor4;
   
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
    * READ_CHAR_TIMEOUT the timeout to wait for the next character in a
    *                   message, in us
    * SERIAL_RESPONSE_TIMEOUT The timeout to wait for a response to our
    *                         query, in us
    */
   double LOOP_RATE = 10.0;
   double ORDER_TIMEOUT = 0.25;

   int BAUDRATE = B115200;
   char* SERIAL_DEV = "/dev/ttyUSB0";
   int READ_CHAR_TIMEOUT = 30;
   int SERIAL_RESPONSE_TIMEOUT = 100;
   char START_CHAR = '`';

   /********************
    * Set up ROS node
    */
   ros::init(argc, argv, "TableMotorToSerial");
   ros::NodeHandle n;

   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = n.subscribe("DriveTable", 2, callbackTableMotorToSerial ); 

   ros::Publisher pub = n.advertise<alfred_msg::TableEncoders>("TableEncoders", 2);

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
         tableOrder_serialArduino1 = 0.0;
         tableOrder_serialArduino2 = 0.0;
         tableOrder_serialArduino3 = 0.0;
         tableOrder_serialArduino4 = 0.0;
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){

         tableOrder_serialArduino1 = 0.0;
         tableOrder_serialArduino2 = 0.0;
         tableOrder_serialArduino3 = 0.0;
         tableOrder_serialArduino4 = 0.0;
         ROS_ERROR( "Base Motor to Serial hasn't received a new motor order in %f mS!", 1000 * ORDER_TIMEOUT );
      }
      // Safety check passed, send 
    
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( receivedOrder_serialArduino == true ){
         numLoopsWithoutOrder = 0;
         receivedOrder_serialArduino = false;
      }

      ROS_INFO( "Sending output to table: [%f, %f, %f, %f]",
                  tableOrder_serialArduino1,
                  tableOrder_serialArduino2,
                  tableOrder_serialArduino3,
                  tableOrder_serialArduino4);
 
      //write
      std::stringstream ss;
      ss << "`" 
         << tableOrder_serialArduino1 << ";"
         << tableOrder_serialArduino2 << ";"
         << tableOrder_serialArduino3 << ";"
         << tableOrder_serialArduino4 << ";\0";
      std::string msg = ss.str();
   
      int isok = serialport_write(fd, msg.c_str() );

      // now read, and wait for the arduino to reply with the encoder values
      int encoders[4];
      char buf[10];

      // First, wait for a response      
      if( serialport_read_until(fd, buf, START_CHAR, SERIAL_RESPONSE_TIMEOUT) <= 0)
      {
         ROS_ERROR("ERROR: Timeout on serial response message start!");
         for( int i(0); i < 4; i++ )
            encoders[i]=-1;
      }
      else{
         for( int i(0); i < 4; i++ ){
            isok = serialport_read_until(fd, buf, ';',  READ_CHAR_TIMEOUT );
            if( isok <= 0 ){
               ROS_ERROR("ERROR: Timeout reading message");
               break;
            }
            encoders[i] = std::atoi(buf);
         }
         if( isok <= 0 )
            for( int i(0); i < 4; i++ )
               encoders[i]=-1;
      }

      ROS_INFO( "Publishing Encoder Vals: [%d, %d, %d, %d]",
                  encoders[0],
                  encoders[1],
                  encoders[2],
                  encoders[3]);

      alfred_msg::TableEncoders encTopic;
      encTopic.enc1 = encoders[0];
      encTopic.enc2 = encoders[1];
      encTopic.enc3 = encoders[2];
      encTopic.enc4 = encoders[3];

      pub.publish(encTopic);
   
      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }

   serialport_close(fd);
   return 0;
}




