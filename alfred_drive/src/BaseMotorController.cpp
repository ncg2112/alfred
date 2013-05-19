/*
 *  BaseMotorController
 *
 * Nathan Grubb
 * April 2013
 */
   
// ROS
#include <ros/ros.h>
#include <alfred_msg/DriveBase.h>
#include <alfred_msg/FollowCommand.h>
#include <alfred_msg/FSRDirection.h>

// CPP
#include <cmath>

double followMotor1;
double followMotor2;
double followMotor3;

bool updatedFollow;

double fsrX;
double fsrY;
double speedPot;
int rotate;
bool follow;

bool updatedDir;

void callbackFollowCommand( const boost::shared_ptr<alfred_msg::FollowCommand const>& followCmd ){
   
   ROS_INFO( "Received Follow Command: [%f, %f, %f]",
               followCmd->motor1,
               followCmd->motor2,
               followCmd->motor3);

   followMotor1 = followCmd->motor1;
   followMotor2 = followCmd->motor2;
   followMotor3 = followCmd->motor3;

   updatedFollow = true;
}

void callbackFsrDirection( const boost::shared_ptr<alfred_msg::FSRDirection const>& fsrDir ){

   ROS_DEBUG( "Received FSR Direction : [%f, %f] speed [%f] rotate [%d] follow [%d]",
               fsrDir->x,
               fsrDir->y,
               fsrDir->speed,
               fsrDir->rotate,
               fsrDir->follow );

   fsrX = fsrDir->x;
   fsrY = fsrDir->y;
   speedPot = fsrDir->speed;
   rotate = fsrDir->rotate;
   follow = fsrDir->follow;
   
   updatedDir = true;
}


int main(int argc, char **argv)
{
   /***************************************
    * Driving Params
    *
    *   The speeds are on a scale from -1 to 1
    *
    *
    *   Thetas define the direction in which the wheel drives, in degrees. 
    *   Clockwise is forward as are the motor numbers, starting at the front
    *   right
    *
    *  MAGNITUDE_THRESHOLD       The smallest magnitude of the ring vector
    *                            that will drive.
    */ 
   double LOOP_RATE = 30.0;
   double ORDER_TIMEOUT = 0.25;
  
   #define PI 3.14159
   double THETA_M1 = -60.0 * PI / 180.0;    // In degrees
   double THETA_M2 = 60.0 * PI / 180.0;
   double THETA_M3 = 180.0 * PI / 180.0;

   double MOTOR_MULTIPLIER[3] = {1,1,1};

   double MAX_ACCELERATION = 1000;      // In % per second
   double MAX_ACC_PER_LOOP = MAX_ACCELERATION / LOOP_RATE;

   double MAX_SPEED = 50;
   double MIN_SPEED = 10;

   double ROTATION_SPEED = 3;

   double MAGNITUDE_THRESHOLD = 0.4;

   /******** ROS Setup *********/
   ros::init(argc, argv, "BaseMotorController");
   ros::NodeHandle node;
  
   ros::Publisher drive_pub = node.advertise<alfred_msg::DriveBase>("DriveBase", 2);
  
   ROS_INFO("Subscribing"); 
   ros::Subscriber sub = node.subscribe("FSRDirection", 2, callbackFsrDirection ); 
   ros::Subscriber sub2 = node.subscribe("FollowCommand", 2, callbackFollowCommand ); 
 
   ros::Rate rate(10.0);
   int maxNumLoopsWithoutOrder = (int)( ORDER_TIMEOUT * LOOP_RATE );
   int numLoopsWithoutFollow(-1);
   int numLoopsWithoutDir(-1);
   
      
   /******** Loop variables Setup *********/
   
   // motor target speeds and current speeds
   double targetM1(0.0);
   double targetM2(0.0);
   double targetM3(0.0);

   double motor1(0.0);
   double motor2(0.0);
   double motor3(0.0);

   std::string torsoName("");
   while(node.ok()){

      double speedRange = MAX_SPEED - MIN_SPEED;
      double speed = speedPot * speedRange + MIN_SPEED; 

      if( follow ){
         // DRIVE FROM KINECT FOLLOW
         if( numLoopsWithoutFollow == -1 ){
            targetM1=0;               
            targetM2=0;               
            targetM3=0;
         }
         else if( numLoopsWithoutFollow >= maxNumLoopsWithoutOrder ){
            targetM1=0;               
            targetM2=0;               
            targetM3=0;
            ROS_ERROR("ERROR: Base Motor Controller, in follow mode, has not received a follow motor order in %f mS!", 1000*ORDER_TIMEOUT);
         }
         else{
            // Safety checks passed
            targetM1 = followMotor1*speed;
            targetM2 = followMotor2*speed;
            targetM3 = followMotor3*speed;
         }
         if( numLoopsWithoutFollow >= 0 ) 
            numLoopsWithoutFollow++;
         if( updatedFollow == true ){
            numLoopsWithoutFollow = 0;
            updatedFollow = false;
         }

         ROS_INFO("Following at [%f, %f, %f]", targetM1, targetM2, targetM3);
      }
      else{
         // DRIVE FROM FSR DIRECTION
         if( numLoopsWithoutDir == -1 ){
            targetM1=0;               
            targetM2=0;               
            targetM3=0;
         }
         else if( numLoopsWithoutDir >= maxNumLoopsWithoutOrder ){
            targetM1=0;               
            targetM2=0;               
            targetM3=0;
            ROS_ERROR("ERROR: Base Motor Controller, in follow mode, has not received a follow motor order in %f mS!", 1000*ORDER_TIMEOUT);
         }
         else{
            // Safety checks passed

            double desiredAngle = std::atan2(fsrY, fsrX);
            double desiredMagnitude = std::sqrt( fsrX*fsrX + fsrY*fsrY );

            ROS_INFO("Table Angle : %f   Magnitude : %f ",desiredAngle*180/PI, desiredMagnitude);
            if( desiredMagnitude < MAGNITUDE_THRESHOLD ){
               targetM1=0;               
               targetM2=0;               
               targetM3=0;
            }
            else{
      
      
               //First, get the angle between the desired vector, which
               // is in FSRx and FSRy
               double angleFromDesired[3];

               angleFromDesired[0] = desiredAngle - THETA_M1; 
               angleFromDesired[1] = desiredAngle - THETA_M2; 
               angleFromDesired[2] = desiredAngle - THETA_M3;

               ROS_INFO("Angle From Desired: [%f, %f, %f]",
                        angleFromDesired[0],
                        angleFromDesired[1],
                        angleFromDesired[2]); 

               targetM1 = std::sin(angleFromDesired[0]) * speed * -1;
               targetM2 = std::sin(angleFromDesired[1]) * speed * -1;
               targetM3 = std::sin(angleFromDesired[2]) * speed * -1;

               //if( angleFromDesired[0] > PI )
                 // targetM1 *= -1;
               //if( angleFromDesired[1] > PI )
                 // targetM2 *= -1;
               //if( angleFromDesired[2] > PI )
                 // targetM3 *= -1;
            }

         } 

         if( rotate > 0 ){
            targetM1-=ROTATION_SPEED;
            targetM2-=ROTATION_SPEED;
            targetM3-=ROTATION_SPEED;
         }
         if( rotate < 0 ){
           targetM1+=ROTATION_SPEED;
           targetM2+=ROTATION_SPEED;
           targetM3+=ROTATION_SPEED;
         }

         if( numLoopsWithoutDir >= 0 ) 
            numLoopsWithoutDir++;
         if( updatedDir == true ){
            numLoopsWithoutDir = 0;
            updatedDir = false;
         }

         ROS_INFO("FSR Ring Control at [%f, %f, %f]", targetM1, targetM2, targetM3);
      }

      /********************
       * Now that we have a target, either from the follow node or
       * computed here from the direction, we ramp the motors to that value.
       */ 
   
      targetM1 *= MOTOR_MULTIPLIER[0];
      targetM2 *= MOTOR_MULTIPLIER[1];
      targetM3 *= MOTOR_MULTIPLIER[2];

      double diff1 = targetM1 - motor1;
      double diff2 = targetM2 - motor2;
      double diff3 = targetM3 - motor3;

      diff1 = diff1 > 0 ? std::min(diff1, MAX_ACC_PER_LOOP) : std::max(diff1, -1.0*MAX_ACC_PER_LOOP);
      diff2 = diff2 > 0 ? std::min(diff2, MAX_ACC_PER_LOOP) : std::max(diff2, -1.0*MAX_ACC_PER_LOOP);
      diff3 = diff3 > 0 ? std::min(diff3, MAX_ACC_PER_LOOP) : std::max(diff3, -1.0*MAX_ACC_PER_LOOP);

      motor1 = motor1 + diff1;
      motor2 = motor2 + diff2;
      motor3 = motor3 + diff3;

      ROS_INFO( "Target Motor Order: [%f, %f, %f]", targetM1, targetM2, targetM3);
      ROS_INFO( "Publishing Motor Order: [%f, %f, %f]", motor1, motor2, motor3 );
 
      alfred_msg::DriveBase msg;
      msg.motor1 = motor1;
      msg.motor2 = motor2;
      msg.motor3 = motor3;
      drive_pub.publish(msg);

      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }
   return 0;
}
