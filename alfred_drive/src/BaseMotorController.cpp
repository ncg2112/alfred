/*
 * 
 *
*/
   
#include <ros/ros.h>
#include <alfred_msg/DriveBase.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
   /* 
    * Following Params
    *
    *   speeds are measured in percents, where 0% is full reverse, 100% full
    *   forward, and 50% is no power
    *
    *   Thesholds are used to define an arc along a donut where the robot will      *   not move. Distances are in meters ??
    */ 
   double FORWARD_SPEED = .7;
   double ANGULAR_SPEED = .6;
   double REVERSE_ANGULAR_SPEED = .4;

   double FAR_THRESHOLD = 1.5;
   double SIDE_THRESHOLD = .3;
   

   ros::init(argc, argv, "listenerTf");
   ros::NodeHandle node;
  
   ros::Publisher drive_pub = node.advertise<alfred_msg::DriveBase>("DriveBase", 2);
 
   tf::TransformListener listener;
   ros::Rate rate(10.0);
      
   double targetX(-1);
   double targetY(-1);
   double prevTargetX(-1);
   double prevTargetY(-1);

   double speed(.5);
   double angularSpeed(.5);

   while(node.ok()){

      tf::StampedTransform transform;
      try{
         std::vector<std::string> frameIds;
         listener.getFrameStrings( frameIds );

         std::string torsoName("");
         for( uint i(0); i < frameIds.size(); i++ ){
            if( frameIds[i].find("torso") != std::string::npos ){
               torsoName = frameIds[i];
               break;
            }
         }

         if( torsoName.compare("") ){
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
                  speed = .5;

               if( targetY > SIDE_THRESHOLD )
                  angularSpeed = ANGULAR_SPEED;
               else if( targetY < -1 * SIDE_THRESHOLD )
                  angularSpeed = REVERSE_ANGULAR_SPEED;
               else
                  angularSpeed = .5;
            }
            else{
               ROS_WARN("Target values not updating\n");
               speed = .5;
               angularSpeed = .5;
            }
            
         }
      }
      catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         continue;
      }
   
      ROS_INFO( "Speed: %f   Angular Speed:  %f", speed, angularSpeed );
      ROS_INFO( "Sending [%f,%f,%f]", 1.0, 1.0, 0.0 );

      /* TODO
       * Calculate the outputs to the motors here
       */ 
      alfred_msg::DriveBase msg;
      msg.motor1 = 1;
      msg.motor2 = 1;
      msg.motor3 = 0;
      drive_pub.publish(msg);

      ROS_INFO("\n");
      rate.sleep();
   }
   return 0;
}
