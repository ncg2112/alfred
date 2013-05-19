/*
 * 
 *
*/
   
// ROS
#include <ros/ros.h>
#include <alfred_msg/DriveBase.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

// CPP
#include <cmath>

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
   double FORWARD_SPEED = 25;
   double ANGULAR_SPEED = 10;
   double REVERSE_ANGULAR_SPEED = -10;

   double FAR_THRESHOLD = 1.5;
   double SIDE_THRESHOLD = .3;
  
   double THETA_M1 = -45.0;            // In degrees
   double THETA_M2 = 45.0;
   double THETA_M3 = 180.0;

   double MAX_ACCELERATION = 8;      // In % per second
   double LOOP_RATE = 10.0;            // In Hz
   double MAX_ACC_PER_LOOP = MAX_ACCELERATION / LOOP_RATE;


   // Projection of the forward-facing unti vector onto each wheel vector
   double sinM1 = std::sin( M_PI * THETA_M1 / 180.0 );
   double sinM2 = std::sin( M_PI * THETA_M2 / 180.0 );
   double sinM3 = std::sin( M_PI * THETA_M3 / 180.0 );
 
   /******** ROS Setup *********/
   ros::init(argc, argv, "listenerTf");
   ros::NodeHandle node;
  
   ros::Publisher drive_pub = node.advertise<alfred_msg::DriveBase>("DriveBase", 2);
 
   tf::TransformListener listener;
   ros::Rate rate(10.0);
      
   /******** Loop variables Setup *********/
   // Targets refer to torso position
   double targetX(-1);
   double targetY(-1);
   double prevTargetX(-1);
   double prevTargetY(-1);

   // base speed and rotation
   double speed(0.0);
   double angularSpeed(0.0);

   // motor target speeds and current speeds
   double targetM1(0.0);
   double targetM2(0.0);
   double targetM3(0.0);

   double motor1(0.0);
   double motor2(0.0);
   double motor3(0.0);

   std::string torsoName("");
   int torsoNum(0);
   bool updating(true);
   while(node.ok()){

      tf::StampedTransform transform;
      try{
         std::vector<std::string> frameIds;
         listener.getFrameStrings( frameIds );

         if( torsoName.compare("") ){

            /****************
             * First, we get a transform and calculate how to move (speed and angular speed)
             */
            listener.lookupTransform("/openni_depth_frame",torsoName,ros::Time(0), transform);

            ROS_INFO("Torso \'%s\' is at [%5f, %5f]", torsoName.c_str(), transform.getOrigin().x(), transform.getOrigin().y() );
            prevTargetX = targetX;
            prevTargetY = targetY;
            targetX = transform.getOrigin().x();
            targetY = transform.getOrigin().y();

            // check to see if we're getting an updated transform
            if( prevTargetX != targetX && prevTargetY != targetY ){
               ROS_INFO("Vaid Target"); 
               if( targetX > FAR_THRESHOLD)
                  speed = FORWARD_SPEED;
               else
                  speed = 0.0;

               if( targetY > SIDE_THRESHOLD )
                  angularSpeed = ANGULAR_SPEED;
               else if( targetY < -1 * SIDE_THRESHOLD )
                  angularSpeed = REVERSE_ANGULAR_SPEED;
               else
                  angularSpeed = 0.0;
            }
            else{
               updating = false;
               torsoName = "";
               speed = 0.0;
               angularSpeed = 0.0;
            }
            if( ! updating )
               ROS_WARN("Target values not updating\n");
               
         }
         else{

            if( torsoNum >= frameIds.size() )
               torsoNum = 0;

            ROS_INFO("   LOOKING for new torso ");

            // find a new torso to track
            torsoName = "";
            for( int i(++torsoNum); i < frameIds.size(); i++ ){
               if( frameIds[i].find("torso") != std::string::npos ){
                  torsoName = frameIds[i];
                  break;
               }
            }
            if( torsoName.compare("") )
               updating = true;
         }
      }
      catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         continue;
      }
   
      ROS_INFO( "Speed: %f   Angular Speed:  %f", speed, angularSpeed );

      /* 
       * Now, we go from the speed to motor orders
       *    This includes ramping motors, and eventually a PID loop
       */

      // First we project the speed forward onto each wheel, this is
      //   simply a projection of the foward onto the wheel vector
      targetM1 = (sinM1 * speed);
      targetM2 = (sinM2 * speed);
      targetM3 = (sinM3 * speed);
     
      // now add the rotation
      targetM1 = targetM1 - angularSpeed; 
      targetM2 = targetM2 - angularSpeed; 
      targetM3 = targetM3 - angularSpeed; 
      
      // now we go from the target velocity to the current velocity
      //  This is just a ramp function
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
   }
   return 0;
}
