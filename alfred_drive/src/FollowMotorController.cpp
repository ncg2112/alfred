/*
 * FollowMotorController
 * 
 * Nathan Grubb
 * April 2013
 *
 *
 * FollowMotorController ROS Noded
 *
 * This Node listens to the transforms the openni_tracker node publishes which
 * are the coordinates of each joint in all the skeletons being tracked.
 *
 * It selects the right torso to track, defined as the lowest torso index that
 * has been updated within a timeout. The openni_tracker labels torsos by an index
 * from 1 to n, torso1 torso2 etc.
 *
 * Once it has received an updated transform from the correct torso it runs a
 * simple algorithm to determine the forward and angular speed of the robot.
 * It defines an arc of a donought as the target area, where it will not move if
 * the torso is within. Otherwise, it sets the forward or angular velocity to a
 * to a constant value in the correct direction to move the torso into that arc.
 *
 * Once the forward and angular speeds are determined it projects those onto
 * each motor, and publishes those as a FollowCommand. The motor outputs are
 * not ramped in this node, but in the BaseMotorController node.
 *
 */
   
// ROS
#include <ros/ros.h>
#include <alfred_msg/FollowCommand.h>
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
   double FORWARD_SPEED = 1;
   double ANGULAR_SPEED = 3;
   double REVERSE_ANGULAR_SPEED = -3;

   double FAR_THRESHOLD = 1.5;
   double SIDE_THRESHOLD = .3;
  
   double THETA_M1 = 60.0;            // In degrees
   double THETA_M2 = -60.0;
   double THETA_M3 = 180.0;

   double MAX_ACCELERATION = 8;      // In % per second       NOT USED - FROM EARLIER VERSION - used to ramp speeds
   double LOOP_RATE = 30.0;            // In Hz
   double MAX_ACC_PER_LOOP = MAX_ACCELERATION / LOOP_RATE; // NOT USED - FROM EARLEIR VERSION - used to ramp speeds


   // Projection of the forward-facing unti vector onto each wheel vector
   double sinM1 = std::sin( M_PI * THETA_M1 / 180.0 );
   double sinM2 = std::sin( M_PI * THETA_M2 / 180.0 );
   double sinM3 = std::sin( M_PI * THETA_M3 / 180.0 );
 
   /******** ROS Setup *********/
   ros::init(argc, argv, "listenerTf");
   ros::NodeHandle node;
  
   ros::Publisher drive_pub = node.advertise<alfred_msg::FollowCommand>("FollowCommand", 2);
 
   tf::TransformListener listener;
   ros::Rate rate(LOOP_RATE);
      
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

         // If we have a valid torso name
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
            // if the transform isn't updated, this is not a valid torso
            else{
               updating = false;
               torsoName = "";
               speed = 0.0;
               angularSpeed = 0.0;
            }
            if( ! updating )
               ROS_WARN("Target values not updating\n");
               
         }
         // If we don't have a valid torso name
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
      targetM1 = (sinM1 * speed * -1);
      targetM2 = (sinM2 * speed * -1);
      targetM3 = (sinM3 * speed);
     
      // now add the rotation
      targetM1 = targetM1 - angularSpeed; 
      targetM2 = targetM2 - angularSpeed; 
      targetM3 = targetM3 - angularSpeed; 
     
      motor1 = targetM1;
      motor2 = targetM2;
      motor3 = targetM3;
      
      ROS_INFO( "Target Motor Order: [%f, %f, %f]", targetM1, targetM2, targetM3);
      ROS_INFO( "Publishing Motor Order: [%f, %f, %f]", motor1, motor2, motor3 );
 
      alfred_msg::FollowCommand msg;
      msg.motor1 = motor1;
      msg.motor2 = motor2;
      msg.motor3 = motor3;
      drive_pub.publish(msg);

      ROS_INFO("\n");
      rate.sleep();
   }
   return 0;
}
