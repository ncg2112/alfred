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
double baseOrder_serialArduino1;
double baseOrder_serialArduino2;
double baseOrder_serialArduino3;

double tableOrder_serialArduino1;
double tableOrder_serialArduino2;
double tableOrder_serialArduino3;
double tableOrder_serialArduino4;

bool receivedOrder_serialArduino;

void callbackBaseMotorToSerial( const boost::shared_ptr<alfred_msg::DriveBase const>& driveOrder){

   ROS_DEBUG( "Received Base Order: [%f,%f,%f]", driveOrder->motor1, driveOrder->motor2, driveOrder->motor3);
   
   baseOrder_serialArduino1 = driveOrder->motor1;
   baseOrder_serialArduino2 = driveOrder->motor2;
   baseOrder_serialArduino3 = driveOrder->motor3;
   
   receivedOrder_serialArduino = true;
}

void callbackTableMotorToSerial( const boost::shared_ptr<alfred_msg::DriveTable const>& driveOrder){

   ROS_DEBUG( "Received Table Order: [%f,%f,%f,%f]", driveOrder->motor1, driveOrder->motor2, 
                                                    driveOrder->motor3, driveOrder->motor4);

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
    */
   double LOOP_RATE = 200.0;
   double ORDER_TIMEOUT = 1;

   int BAUDRATE = B9600;//B115200;
   char* SERIAL_DEV = "/dev/arduino";

   /********************
    * Set up ROS node
    */
   ros::init(argc, argv, "BaseMotorToSerial");
   ros::NodeHandle n;

   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = n.subscribe("DriveBase", 2, callbackBaseMotorToSerial ); 
   ros::Subscriber sub2 = n.subscribe("DriveTable", 2, callbackTableMotorToSerial ); 

   ROS_INFO( "Starting Publisher" );
   ros::Publisher enc_pub = n.advertise<alfred_msg::TableEncoders>("TableEncoders", 2);

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
         baseOrder_serialArduino1 = 0.0;
         baseOrder_serialArduino2 = 0.0;
         baseOrder_serialArduino3 = 0.0;

         tableOrder_serialArduino1 = 0.0;
         tableOrder_serialArduino2 = 0.0;
         tableOrder_serialArduino3 = 0.0;
         tableOrder_serialArduino4 = 0.0;
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){

         baseOrder_serialArduino1 = 0.0;
         baseOrder_serialArduino2 = 0.0;
         baseOrder_serialArduino3 = 0.0;
         tableOrder_serialArduino1 = 0.0;
         tableOrder_serialArduino2 = 0.0;
         tableOrder_serialArduino3 = 0.0;
         tableOrder_serialArduino4 = 0.0;
         ROS_ERROR( "All Motor to Serial hasn't received a new motor order in %f mS!", 1000 * ORDER_TIMEOUT );
      }
      // Safety check passed, send 
    
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( receivedOrder_serialArduino == true ){
         numLoopsWithoutOrder = 0;
         receivedOrder_serialArduino = false;
      }

      ROS_INFO( "Sending output to base: [%f, %f, %f : %f, %f, %f, %f]",
                  baseOrder_serialArduino1,
                  baseOrder_serialArduino2,
                  baseOrder_serialArduino3,
                  tableOrder_serialArduino1,
                  tableOrder_serialArduino2,
                  tableOrder_serialArduino3,
                  tableOrder_serialArduino4);
 
      //write
      std::stringstream ss;
      ss << "`"
         << baseOrder_serialArduino1 << ";"
         << baseOrder_serialArduino2 << ";"
         << baseOrder_serialArduino3 << ";"
         << tableOrder_serialArduino1 << ";"
         << tableOrder_serialArduino2 << ";"
         << tableOrder_serialArduino3 << ";"
         << tableOrder_serialArduino4 << ";\0";
      std::string msg = ss.str();
   
      ROS_INFO("Sending over serial\'%s\'", msg.c_str() );

      serialport_write(fd, msg.c_str() );
      
      
      // Now read the encoder positions
      bool isok(true);
      char buf[4][10];
      isok = serialport_read_until(fd, buf[0], '`', 100 ) > 0;
      if( ! isok ){
         ROS_WARN("ERROR: Timeout on response from arduino");
         continue;
      }  
      for(int i(0); i < 4; i++ ){
         int charRead = serialport_read_until(fd, buf[i], ';', 100);
         if( charRead < 0 ){
            ROS_WARN("ERROR: Arduino halted mid-response or incorrect format");
            isok = false;
         }
      }

      double encPos[4];
      for(int i(0); i < 4; i++ ){
         if(isok){
            //ROS_INFO("Recv string %d \'%s\'", i,buf[i]);
            ROS_INFO("Enc Pos %d : %f", i, std::atof(buf[i]));
            encPos[i] = std::atof(buf[i]);
         }
      }
   
      if(isok){

         alfred_msg::TableEncoders encMsg;
         encMsg.enc1 = encPos[0];
         encMsg.enc2 = encPos[1];
         encMsg.enc3 = encPos[2];
         encMsg.enc4 = encPos[3];

         enc_pub.publish(encMsg);

      }

 
      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }

   serialport_close(fd);
   return 0;
}




