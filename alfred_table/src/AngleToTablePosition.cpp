/*
 * TableEncoderPID.cpp
 * 
 * Nathan Grubb, April 2013
 *
 * 
 */
   
// ROS
#include <ros/ros.h>
#include <alfred_msg/TablePosition.h>
#include <alfred_msg/TableAngleStatus.h>

// CPP
#include <cmath>

// PID
#include "PID_v1.h"

//double encoderPosition_arduino[4];
double xAccVal(0);
double yAccVal(0);
double zAccVal(0);
int upDown(0);

//bool receivedEncUpdate_arduino;
bool receivedStatusUpdate;

void callbackTableAngleStatusUpdate( const boost::shared_ptr<alfred_msg::TableAngleStatus const>& angleUpdate){
   
   ROS_DEBUG( "Received Table Angle Status [%f, %f, %f, %d]",
               angleUpdate->xAcc,
               angleUpdate->yAcc,
               angleUpdate->zAcc,
               angleUpdate->upDown );

   xAccVal = angleUpdate->xAcc;
   yAccVal = angleUpdate->yAcc;
   zAccVal = angleUpdate->yAcc;
   upDown = angleUpdate->upDown; 

   receivedStatusUpdate = true;
}

int main(int argc, char **argv)
{
   /* 
    * Balancing Params 
    * 
    *
    *  BY C1 Y0 DY_REST and NUT_OFFSETS are in inches
    * 
    * MAX_HEIGHT and MIN_HEIGHT define the allowed range
    *   of target encoder values.
    *
    * ANLGES ARE IN RADIANS
    */ 

   #define PI 3.14159
   #define GRAVITY 9.8

   double LOOP_RATE = 100.0;
   double UPDATE_ACC_TIMEOUT = 0.4;

   int MAX_HEIGHT = 19000;
   int MIN_HEIGHT = 2600;

   // Table constraint sizes used for angle to nut
   // position calculation
   double BY = 16.875;
   double C1 = 3.22;
   double Y0 = 8;
   double DY_REST = 16.183868;
   
   double NUT_OFFSET[4] = {.9843, .9843, .9843, .9744};

   int COUNTS_PER_INCH = 1829;
   
   double ACC_OFFSET_X = 0;
   double ACC_OFFSET_Y = 0;


   /******** ROS Setup *********/
   ros::init(argc, argv, "AngleToTablePosition");
   ros::NodeHandle node;
  
   ROS_INFO( "Starting Publisher" );
   ros::Publisher drive_pub = node.advertise<alfred_msg::TablePosition>("TablePosition", 2);
   
   ROS_INFO( "Starting Subscriber" );
   ros::Subscriber sub = node.subscribe("TableAngleStatus", 2, callbackTableAngleStatusUpdate ); 
 

   /******** height control variables Setup *********/
   int avgHeight(MIN_HEIGHT);
   int motorPos[4];

   /******** Loop variables Setup *********/
   ros::Rate rate(LOOP_RATE);
   int maxNumLoopsWithoutOrder = (int)( UPDATE_ACC_TIMEOUT * LOOP_RATE );
   int numLoopsWithoutOrder(-1);
  
   while(node.ok()){
    
      // The first time we recieve acc, use that as level
      if( numLoopsWithoutOrder == -1 && receivedStatusUpdate ){
   
         ACC_OFFSET_X = xAccVal; 
         ACC_OFFSET_Y = yAccVal; 

      }
         
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( receivedStatusUpdate == true ){
         numLoopsWithoutOrder = 0;
         receivedStatusUpdate = false;
      }
      
      // TODO get angle from accelerometer!
      // ANGLES ARE IN RADIANS    

      double aX = (xAccVal - ACC_OFFSET_X) / 163.84; 
      double aY = (yAccVal - ACC_OFFSET_Y) / 163.84 * -1;

      double angleX = std::asin(aX);
      double angleY = std::asin(aY);

      ROS_INFO("DefAcc  : [%f, %f]", ACC_OFFSET_X, ACC_OFFSET_Y);
      ROS_INFO("AccVal  : [%f, %f]", xAccVal, yAccVal);
      ROS_INFO("Real Acc: [%f, %f]", aX, aY);
      ROS_INFO("Angle   : [%f, %f]", angleX * 180 / PI, angleY * 180 / PI);

      // calculate offset
   
      int deltaX(0);
      int deltaY(0);

      //if( angleX < -1 * PI / 180 || angleX > 1 * PI / 180 )
      {
         angleX += PI / 2;
         double dy_x = std::sqrt( BY*BY - std::pow(C1 - Y0 * std::sin(angleX),2) ) + Y0 * std::cos(angleX);
         dy_x -= DY_REST;
         //dy_x /= 2;
         dy_x *= 3;
         deltaX = (int)(dy_x * COUNTS_PER_INCH );      
      }  

      //if( angleY < -1 * PI / 180 || angleY > 1 * PI / 180 )
      {
         angleY += PI / 2;
         double dy_y = std::sqrt( BY*BY - std::pow(C1 - Y0 * std::sin(angleY),2) ) + Y0 * std::cos(angleY);
         dy_y -= DY_REST;
         //dy_y /= 2;
         dy_y *= 3;
         deltaY = (int)(dy_y * COUNTS_PER_INCH );
      }

   /*
      // Get deltaX and deltaY a different way to see if above formula is wrong
      //angleX -= PI/2;
      angleX *= -1;
      double R = 8;// from center to arm joint
      double r = 3.22;// offset from center of shaft to nut joint
      double l = 16.875;// length of arm
      double beta = PI/2 - angleX;
      double r_prime = R * std::sin(beta);
      double l_bar = l * (1 + r / (r_prime - r));
      double L2 = R * std::cos(PI/2 - angleX) + std::sqrt(R*R*(std::pow(std::cos(PI/2-angleX),2)-1)+l_bar*l_bar);
      double L1 = R * std::cos(PI/2 + angleX) + std::sqrt(R*R*(std::pow(std::cos(PI/2+angleX),2)-1)+l_bar*l_bar);
      double dx = (L2 - L1)/2;
      //ROS_INFO("Old deltaX: %f  -> %i", dy_x, deltaX); 
      int deltaX = (int)(dx * COUNTS_PER_INCH);
      ROS_INFO("New deltaX: %f  -> %i", dx, deltaX);

      //angleY -= PI/2;
      angleY *= -1;
      beta = PI/2 + angleY;
      r_prime = R * std::sin(beta);
      l_bar = l * (1 + r / (r_prime - r));
      L2 = R * std::cos(PI/2 - angleY) + std::sqrt(R*R*(std::pow(std::cos(PI/2-angleY),2)-1)+l_bar*l_bar);
      L1 = R * std::cos(PI/2 + angleY) + std::sqrt(R*R*(std::pow(std::cos(PI/2+angleY),2)-1)+l_bar*l_bar);
      double dy = (L2 - L1)/2;
      //ROS_INFO("Old deltaY: %f  -> %i", dy_y, deltaY);
      int deltaY = (int)(dy * COUNTS_PER_INCH);
      ROS_INFO("New deltaY: %f  -> %i", dy, deltaY);
   */
      

      //ROS_INFO("AvgHeight: %d   dy_x: %f  deltaX:  %d  deltaY: %d",avgHeight, dy_x, deltaX, deltaY);

      // add offsets, while checking signs corresponding to angle signs
      if( angleX > 0){
         motorPos[0] = avgHeight + deltaX - NUT_OFFSET[0];
         motorPos[2] = avgHeight - deltaX - NUT_OFFSET[2];
      }
      else{
         motorPos[0] = avgHeight - deltaX - NUT_OFFSET[0];
         motorPos[2] = avgHeight + deltaX - NUT_OFFSET[2];
      }

      if( angleY > 0){
         motorPos[1] = avgHeight + deltaY - NUT_OFFSET[1];
         motorPos[3] = avgHeight - deltaY - NUT_OFFSET[3];
      }
      else{
         motorPos[1] = avgHeight - deltaY - NUT_OFFSET[1];
         motorPos[3] = avgHeight + deltaY - NUT_OFFSET[3];
      }

      // Now, if the table should move up/down,
      // change the average height

      if( upDown == -1 )
         avgHeight -= 100;
      if( upDown == 1 )
         avgHeight += 100;

      if( avgHeight > MAX_HEIGHT )
         avgHeight = MAX_HEIGHT;
      if( avgHeight < MIN_HEIGHT )
         avgHeight = MIN_HEIGHT;


      // Now do some safety checks, and turn off motors if they fail
      // If we haven't yet started
      if( numLoopsWithoutOrder == -1){
         for(int i(0); i < 4; i++ )
            motorPos[i] = 0;
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){
         for(int i(0); i < 4; i++ )
            motorPos[i] = 0;
         ROS_ERROR( "Angle To Table hasn't received an accelerometer update in %f mS!", 1000 * UPDATE_ACC_TIMEOUT );
      }

      // Safety check passed, continue


      ROS_INFO( "Table Position Order: [%d, %d, %d, %d]",
                 motorPos[0],
                 motorPos[1],
                 motorPos[2],
                 motorPos[3] );
 
      alfred_msg::TablePosition msg;
      msg.enc1 = motorPos[0];
      msg.enc2 = motorPos[1];
      msg.enc3 = motorPos[2];
      msg.enc4 = motorPos[3];
      drive_pub.publish(msg);

      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }
   return 0;
}
