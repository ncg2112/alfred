/*
 * ReadTableFromSerial.cpp
 * 
 * Nathan Grubb, April 2013
 *
 * Reads the UART over USB messages from the PIC, and pulished
 * the table status messages including ring direction status and
 * table angle status 
 */
   
// ROS
#include <ros/ros.h>
#include <alfred_msg/TableAngleStatus.h>
#include <alfred_msg/FSRDirection.h>

// CPP
#include <cmath>

// serial
#include <alfred_msg/SerialIO.h>

//double encoderPosition_arduino[4];
double xAccVal_PIC(0);
double yAccVal_PIC(0);
int upDown_PIC(0);


int main(int argc, char **argv)
{
   /********************** 
    * Table Read Status Params
    * 
    * 
    * BAUDRATE          baudrate from PIC 
    * SERIAL_DEV        the default dev file pointing to the PIC
    * 
    * INITIAL_TIMEOUT   timeout to hear beginning of message after request
    * CHAR_TIMEOUT      timeout after the message starts for each char
    */ 
 
   int BAUDRATE = B115200;
   char* SERIAL_DEV = "/dev/pic";

   double LOOP_RATE = 100.0;
   double UPDATE_PIC_TIMEOUT = 0.25;

   char REQUEST_CHAR = '$';
   char BEGIN_MSG_CHAR = '`';
   char DELIM_CHAR = ';';

   // usec
   double INITIAL_TIMEOUT = 200;
   double CHAR_TIMEOUT = 40;

   /******** ROS Setup *********/
   ros::init(argc, argv, "ReadTableFromSerial");
   ros::NodeHandle node;
  
   ROS_INFO( "Starting Publisher" );
   ros::Publisher angle_pub = node.advertise<alfred_msg::TableAngleStatus>("TableAngleStatus", 2);
   ros::Publisher dir_pub = node.advertise<alfred_msg::FSRDirection>("FSRDirection", 2);
   

   std::string debug("d");
   bool DEBUG = ( argc > 2 && debug.compare(argv[2]) == 0 );
   if(DEBUG)
      ROS_INFO("Printing all from serial debug");

   /******** Start Serial *********/
   char* serialDevice = SERIAL_DEV;
   if( argc > 1 )
      serialDevice = argv[1];

   int fd = serialport_init(serialDevice, BAUDRATE);
   if(fd < 0){
      ROS_ERROR( "Could not open serial device \'%s\'\n", serialDevice); 
      return -1;
   }
   usleep( 3000 * 1000 );

   /******** Loop variables Setup *********/
   ros::Rate rate(LOOP_RATE);
   int maxNumLoopsWithoutOrder = (int)( UPDATE_PIC_TIMEOUT * LOOP_RATE );
   int numLoopsWithoutOrder(-1);
   bool receivedStatusUpdate(false);
  
   while(node.ok()){

      // If we haven't yet started
      if( numLoopsWithoutOrder == -1){
         // do anything here?
      }
      // Safety Check 1 : time without orders
      else if( numLoopsWithoutOrder >= maxNumLoopsWithoutOrder ){
         // do anything here?
         ROS_ERROR( "ERROR Table Serial Status hasn't received a message from the PIC in %f mS!", 1000 * UPDATE_PIC_TIMEOUT );
      }

      // Safety check passed, continue
    
      if( numLoopsWithoutOrder >= 0 ) 
         numLoopsWithoutOrder++;
      if( receivedStatusUpdate == true ){
         numLoopsWithoutOrder = 0;
         receivedStatusUpdate = false;
      }

      if( DEBUG ){
         char b[1];
         int n = read(fd, b, 1);
         if( n != 0 )
            printf("%c", b[0]);
         continue;
      }


      // Request the status
      ROS_INFO("Sending Query");
      std::stringstream ss;
      ss << REQUEST_CHAR;
      std::string msg = ss.str();
      int isok = serialport_write(fd,msg.c_str());  
     
      if( isok <= 0 ){
         ROS_WARN("ERROR: Requesting Table Status from PIC, write returned %d", isok);
         continue;
      }
   
      // Now, read the table status
      // Message Format is `%d;%d;%d;%d;%d;%d;%d;%d;%d;
      //                    dX dY RT UD aX aY aZ S  F   

      int numItems = 9;
      char buf[numItems][128];
     
      char b[1]; 
      isok = serialport_read_until(fd, b, BEGIN_MSG_CHAR, INITIAL_TIMEOUT);
      if( isok <= 0 ){
         ROS_ERROR("ERROR: Timeout on response from PIC!");
         continue;
      }
      ROS_INFO("Got start char");
      bool isGoodRead(true);
      for(int i(0); i < numItems; i++ ){
         isok = serialport_read_until(fd, buf[i], DELIM_CHAR, CHAR_TIMEOUT);
         ROS_INFO("Read in \'%s\'", buf[i]);
         if( isok <= 0 ){
            ROS_WARN("ERROR: PIC Timeout halted mid-response or incorrect format");
            isGoodRead = false;
         }
      }
      if( ! isGoodRead ){
         ROS_ERROR("ERROR: Could not read status from PIC");
         continue;
      }

      receivedStatusUpdate = true;

      double fsrDir[2];
      for( int i(0); i < 2; i++ ){
         int rawFsr = std::atoi(buf[i]);

         // now process, turn from scale 0-255 to a percent from -1 to 1
         fsrDir[i] = (double)rawFsr / 128;      

         ROS_INFO("FSR Direction %d : %f", i, fsrDir[i]);
      } 
      
      double acc[2];
      for( int i(0); i < 3; i++ ){
         ROS_INFO("acc %d raw is \'%s\'", i, buf[i+4] );
         int rawAcc = std::atoi(buf[i+4]);
         acc[i] = (double)rawAcc;
         ROS_INFO("Accelerator %d : %f", i, acc[i]);
      } 

      double speed = (double)(std::atoi(buf[7])) / 255.0;
      ROS_INFO("Speed : %f", speed);

      ROS_INFO("Rotate string is \'%s\'", buf[2]);
      int rotate = std::atoi(buf[2]);
      ROS_INFO("Rotate : %d", rotate);

      ROS_INFO("UpDown string is \'%s\'", buf[3]);
      int upDown = std::atoi(buf[3]);
      ROS_INFO("UpDown : %d", upDown);
      
      int follow = std::atoi(buf[8]);
      ROS_INFO("Follow : %d", follow);


      // now populate the FSR Direction message

      alfred_msg::FSRDirection dirMsg;

      dirMsg.x = fsrDir[0]; 
      dirMsg.y = fsrDir[1];
      dirMsg.speed = speed;
      dirMsg.rotate = rotate;
      dirMsg.follow = follow == 1;

      // now populate the angle status message
   
      alfred_msg::TableAngleStatus angleMsg;
      
      angleMsg.xAcc = acc[0];
      angleMsg.yAcc = acc[1];
      angleMsg.zAcc = acc[2];
      angleMsg.upDown = upDown;

      dir_pub.publish(dirMsg); 
      angle_pub.publish(angleMsg);

      ROS_INFO("\n");
      rate.sleep();
      ros::spinOnce();
   }
   return 0;
}
