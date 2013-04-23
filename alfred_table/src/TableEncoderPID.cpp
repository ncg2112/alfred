/*
 * TableEncoderPID.cpp
 * 
 * Nathan Grubb, April 2013
 *
 * 
 */
   
// ROS
#include <ros/ros.h>
#include <alfred_msg/DriveTable.h>
#include <alfred_msg/TableEncoders.h>

// CPP
#include <cmath>

// PID
#include "PID_v1.h"

double encoderPosition_arduino[4];

bool receivedEncUpdate_arduino;

void callbackTableEncoderUpdate( const boost::shared_ptr<alfred_msg::TableEncoders const>& encUpdate ){

      ROS_DEBUG( "Received Encoder Update: [%d, %d, %d, %d]",
                  encUpdate->enc1,
                  encUpdate->enc2,
                  encUpdate->enc3,
                  encUpdate->enc4 );

      encoderPosition_arduino[0] = encUpdate->enc1;
      encoderPosition_arduino[1] = encUpdate->enc2;
      encoderPosition_arduino[2] = encUpdate->enc3;
      encoderPosition_arduino[3] = encUpdate->enc4;

      receivedEncUpdate_arduino = true;
}

int main(int argc, char **argv)
{
   /* 
    * Following Params
    *
    *   The speeds are on a scale from -1 to 1
    *
    *   Thesholds are used to define an arc along a donut where the robot will      
    *   not move. Distances are in meters ??
    *
    *   Thetas define the direction in which the wheel points, in degrees. 
    *   Clockwise is forward as are the motor numbers, starting at the front
    *   right
    */ 

   double LOOP_RATE = 10.0;
   double UPDATE_ENC_TIMEOUT = 0.25;



   
   /******** ROS Setup *********/
   ros::init(argc, argv, "TableEncoderPID");
   ros::NodeHandle node;
  
   ROS_INFO( "Starting Publisher" );
   ros::Publisher drive_pub = node.advertise<alfred_msg::DriveTable>("DriveTable", 2);
   
   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = node.subscribe("TableEncoders", 2, callbackTableEncoderUpdate ); 
 
   /******** PID variables Setup *********/
   double desiredPos[4] = {0,0,0,0};
   double outputPID[4];

   double kp[4] = {10,0,0,0};
   double ki[4] = {0,0,0,0};
   double kd[4] = {0,0,0,0};

   PID controller[4];

   for(int i(0); i < 4; i++){
      controller[i] = PID(&encoderPosition_arduino[i], &outputPID[i], &desiredPos[i],
                          kp[i], ki[i], kd[i], 1.0 / LOOP_RATE, DIRECT );
      controller[i].SetMode(AUTOMATIC);
      controller[i].SetOutputLimits(-100, 100 );
   }


   /******** Loop variables Setup *********/
   ros::Rate rate(LOOP_RATE);
   int maxNumLoopsWithoutOrder = (int)( UPDATE_ENC_TIMEOUT * LOOP_RATE );
   int numLoopsWithoutOrder(-1);
   
   double motorCmd[4] = {0,0,0,0};


   while(node.ok()){

      // If we haven't yet started
      if( numLoopsWithoutOrder == -1){
         for(int i(0); i < 4; i++ )
            motorCmd[i] = 0;
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){
         for(int i(0); i < 4; i++ )
            motorCmd[i] = 0;
         ROS_ERROR( "Table Encoder PID hasn't received an encoder update in %f mS!", 1000 * UPDATE_ENC_TIMEOUT );
      }

      // Safety check passed, continue
    
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( receivedEncUpdate_arduino == true ){
         numLoopsWithoutOrder = 0;
         receivedEncUpdate_arduino = false;
      }


      // Actually run the PID here!
      for( int i(0); i < 4; i++ )
         controller[i].Compute();

      for( int i(0); i < 4; i++ )
         motorCmd[i] = outputPID[i];

     
      ROS_INFO( "Target Motor Order: [%f, %f, %f, %f]",
                 motorCmd[0],
                 motorCmd[1],
                 motorCmd[2],
                 motorCmd[3] );
 
      alfred_msg::DriveTable msg;
      msg.motor1 = motorCmd[0];
      msg.motor1 = motorCmd[1];
      msg.motor1 = motorCmd[2];
      msg.motor1 = motorCmd[3];
      drive_pub.publish(msg);

      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }
   return 0;
}
